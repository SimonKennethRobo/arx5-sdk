import os
import select
import sys
import termios
import threading
import time
import tty
from queue import Queue

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
from arx5_interface import (
    Arx5CartesianController,
    ControllerConfigFactory,
    EEFState,
    Gain,
    LogLevel,
    RobotConfigFactory,
)
from multiprocessing.managers import SharedMemoryManager

import click

# Terminals only report that a key was typed, not when it's released, so "held"
# state below is approximated: a key counts as held as long as the terminal's
# own key-repeat keeps re-triggering it within this window. Raise this if
# motion stutters between repeats, lower it for a snappier stop on release.
HOLD_TIMEOUT = 0.3

KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_PAGE_UP, KEY_PAGE_DOWN = range(6)

ESCAPE_SEQUENCES = {
    "\x1b[A": KEY_UP,
    "\x1b[B": KEY_DOWN,
    "\x1b[C": KEY_RIGHT,
    "\x1b[D": KEY_LEFT,
    "\x1b[5~": KEY_PAGE_UP,
    "\x1b[6~": KEY_PAGE_DOWN,
}


class KeyboardReader:
    """Approximates held-key state by reading raw keystrokes from the current
    terminal (works over plain SSH, no X server / root access required)."""

    def __init__(self):
        self._last_seen = {}
        self._lock = threading.Lock()
        self._running = False
        self._fd = sys.stdin.fileno()
        self._old_settings = None
        self._thread = None

    def start(self):
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)  # keeps Ctrl+C -> SIGINT, unlike full raw mode
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def _mark(self, key):
        with self._lock:
            self._last_seen[key] = time.monotonic()

    def _read_char(self, timeout):
        ready, _, _ = select.select([self._fd], [], [], timeout)
        if ready:
            return os.read(self._fd, 1).decode(errors="ignore")
        return None

    def _run(self):
        while self._running:
            ch = self._read_char(timeout=0.05)
            if ch is None:
                continue
            if ch == "\x1b":
                seq = ch
                # Arrow / page keys arrive as a fast multi-byte burst; give it a
                # brief window to complete before treating it as a bare escape.
                for _ in range(4):
                    nxt = self._read_char(timeout=0.01)
                    if nxt is None:
                        break
                    seq += nxt
                    if seq in ESCAPE_SEQUENCES:
                        break
                key = ESCAPE_SEQUENCES.get(seq)
                if key is not None:
                    self._mark(key)
            else:
                self._mark(ch)

    def is_held(self, key, now=None):
        now = now if now is not None else time.monotonic()
        with self._lock:
            last = self._last_seen.get(key)
        return last is not None and (now - last) < HOLD_TIMEOUT


def start_keyboard_teleop(controller: Arx5CartesianController):

    ori_speed = 1.0
    pos_speed = 0.4
    gripper_speed = 0.04
    target_pose_6d = controller.get_home_pose()

    target_gripper_pos = 0.0
    cmd_dt = 0.01
    preview_time = 0.1
    window_size = 5
    keyboard_queue = Queue(window_size)
    robot_config = controller.get_robot_config()
    controller_config = controller.get_controller_config()

    print("Teleop tracking started.")
    print(
        "Controls: arrows = x/y, page up/down = z, q/a w/s e/d = roll/pitch/yaw, r/f = gripper, space = reset to home."
    )

    reader = KeyboardReader()
    reader.start()

    def get_filtered_keyboard_output():
        now = time.monotonic()
        state = np.zeros(6, dtype=np.float64)
        if reader.is_held(KEY_UP, now):
            state[0] = 1
        if reader.is_held(KEY_DOWN, now):
            state[0] = -1
        if reader.is_held(KEY_LEFT, now):
            state[1] = 1
        if reader.is_held(KEY_RIGHT, now):
            state[1] = -1
        if reader.is_held(KEY_PAGE_UP, now):
            state[2] = 1
        if reader.is_held(KEY_PAGE_DOWN, now):
            state[2] = -1
        if reader.is_held("q", now):
            state[3] = 1
        if reader.is_held("a", now):
            state[3] = -1
        if reader.is_held("w", now):
            state[4] = 1
        if reader.is_held("s", now):
            state[4] = -1
        if reader.is_held("e", now):
            state[5] = 1
        if reader.is_held("d", now):
            state[5] = -1

        if keyboard_queue.maxsize > 0 and keyboard_queue._qsize() == keyboard_queue.maxsize:
            keyboard_queue._get()

        keyboard_queue.put(state)

        return np.mean(np.array(list(keyboard_queue.queue)), axis=0)

    directions = np.zeros(6, dtype=np.float64)
    start_time = time.monotonic()
    loop_cnt = 0
    try:
        while True:
            eef_state = controller.get_eef_state()
            print(
                f"Time elapsed: {time.monotonic() - start_time:.03f}s, x: {eef_state.pose_6d()[0]:.03f}, y: {eef_state.pose_6d()[1]:.03f}, z: {eef_state.pose_6d()[2]:.03f}",
                end="\r",
            )
            # keyboard state is in the format of (x y z roll pitch yaw)
            prev_directions = directions
            directions = np.zeros(7, dtype=np.float64)
            state = get_filtered_keyboard_output()
            now = time.monotonic()
            key_open = reader.is_held("r", now)
            key_close = reader.is_held("f", now)
            key_space = reader.is_held(" ", now)

            if key_space:
                controller.reset_to_home()
                target_pose_6d = controller.get_home_pose()
                target_gripper_pos = 0.0
                loop_cnt = 0
                start_time = time.monotonic()
                continue
            elif key_open and not key_close:
                gripper_cmd = 1
            elif key_close and not key_open:
                gripper_cmd = -1
            else:
                gripper_cmd = 0

            target_pose_6d[:3] += state[:3] * pos_speed * cmd_dt
            target_pose_6d[3:] += state[3:] * ori_speed * cmd_dt
            target_gripper_pos += gripper_cmd * gripper_speed * cmd_dt
            if target_gripper_pos >= robot_config.gripper_width:
                target_gripper_pos = robot_config.gripper_width
            elif target_gripper_pos <= 0:
                target_gripper_pos = 0
            loop_cnt += 1
            while time.monotonic() < start_time + loop_cnt * cmd_dt:
                pass

            current_timestamp = controller.get_timestamp()
            eef_cmd = EEFState()
            eef_cmd.pose_6d()[:] = target_pose_6d
            eef_cmd.gripper_pos = target_gripper_pos
            eef_cmd.timestamp = current_timestamp + preview_time
            controller.set_eef_cmd(eef_cmd)
    finally:
        reader.stop()


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
def main(model: str, interface: str):
    robot_config = RobotConfigFactory.get_instance().get_config(model)
    controller_config = ControllerConfigFactory.get_instance().get_config(
        "cartesian_controller", robot_config.joint_dof
    )
    controller_config.use_dls_ik = False
    controller = Arx5CartesianController(robot_config, controller_config, interface)
    controller.reset_to_home()

    robot_config = controller.get_robot_config()
    gain = Gain(robot_config.joint_dof)
    controller.set_log_level(LogLevel.DEBUG)
    np.set_printoptions(precision=4, suppress=True)
    try:
        start_keyboard_teleop(controller)
    except KeyboardInterrupt:
        print(f"Teleop recording is terminated. Resetting to home.")
        controller.reset_to_home()
        controller.set_to_damping()


if __name__ == "__main__":
    main()
