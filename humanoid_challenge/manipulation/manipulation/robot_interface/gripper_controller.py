# robot_interface/gripper_controller.py

import threading
import time

from manipulation.robot_interface.gripper_command import (
    GripperCommand,
)


class GripperInterface:

    # Verified against GripperController (reference, works):
    #   0.0 = open  (열림)
    #   1.0 = close (닫힘)
    OPEN = 0.0
    CLOSE = 1.0

    # Position tolerance for open/close state.
    OPEN_THRESHOLD = 0.1
    CLOSE_THRESHOLD = 0.9

    def __init__(
        self,
        node,
        ignore_new_calls_while_executing: bool = True,
    ):

        self._node = node

        self._command = GripperCommand(node)

        self._ignore_new_calls_while_executing = (
            ignore_new_calls_while_executing
        )

        self._is_motion_requested = False
        self._is_executing = False

        self._lock = threading.Lock()

    def __call__(self, side):

        self.toggle(side)

    def _execute(self, fn, *args, **kwargs):

        self._is_motion_requested = False

        try:

            fn(*args, **kwargs)

        finally:

            with self._lock:
                self._is_executing = False

    def _start_execution(self, fn, *args, **kwargs):

        thread = threading.Thread(
            target=self._execute,
            args=(fn,) + args,
            kwargs=kwargs,
            daemon=True,
        )

        thread.start()

    def toggle(self, side):

        if self.is_open(side):

            self.close(
                side,
                skip_if_noop=False,
            )

        else:

            self.open(
                side,
                skip_if_noop=False,
            )

    def control(
        self,
        side,
        position: float,
    ) -> bool:

        side = side.lower()

        with self._lock:
            if (
                self._ignore_new_calls_while_executing
                and self._is_executing
            ):
                self._log.warn(
                    f'[GripperInterface.control] [{side}] dropped — still executing'
                )
                return False
            self._is_motion_requested = True
            self._is_executing = True

        self._start_execution(
            self._command.send,
            side,
            position,
        )
        return True

    def open(
        self,
        side,
        skip_if_noop=False,
    ):

        side = side.lower()

        if skip_if_noop and self.is_open(side):
            return

        self.control(
            side,
            self.OPEN,
        )

    def open_to(self, side, amount: float) -> bool:
        """Open gripper to a specific amount (0.0 = fully open, 1.0 = fully closed).

        Use when starting from closed state and opening just enough to fit around an object.

        Examples
        --------
        gripper.open_to('right', 0.8)   # 닫힌 상태에서 살짝만 열기
        gripper.open_to('right', 0.5)   # 절반 열기
        """
        side = side.lower()
        amount = max(self.OPEN, min(self.CLOSE, amount))
        return self.control(side, amount)
        
    def close(
        self,
        side,
        skip_if_noop=False,
    ):

        side = side.lower()

        if skip_if_noop and self.is_closed(side):
            return

        self.control(
            side,
            self.CLOSE,
        )

    def wait_motion(self):
        """Block until the physically commanded motion completes."""
        self._command.wait_motion()

    def wait_until_executed(self):

        if (
            not self._is_motion_requested
            and not self._is_executing
        ):
            return False

        while (
            self._is_motion_requested
            or self._is_executing
        ):
            time.sleep(0.001)

        return True

    def force_reset_executing_state(self):

        with self._lock:
            self._is_motion_requested = False
            self._is_executing = False

    def is_open(self, side):

        side = side.lower()

        position = self._command.get_position(
            side
        )

        if position is None:
            return True

        return position <= self.OPEN_THRESHOLD

    def is_closed(self, side):

        side = side.lower()

        position = self._command.get_position(
            side
        )

        if position is None:
            return False

        return position >= self.CLOSE_THRESHOLD