#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys

from gui_control.gui_control import main


def _append_default_ros_args() -> None:
    """Append default ROS params for right-hand L6 GUI control."""
    default_args = [
        "--ros-args",
        "-p",
        "hand_type:=right",
        "-p",
        "hand_joint:=L6",
        "-p",
        "topic_hz:=30",
        "-p",
        "is_arc:=false",
    ]
    sys.argv.extend(default_args)


if __name__ == "__main__":
    _append_default_ros_args()
    main()
