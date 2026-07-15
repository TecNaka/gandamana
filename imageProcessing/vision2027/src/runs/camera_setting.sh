#!/bin/bash
v4l2-ctl -c brightness=97
v4l2-ctl -c contrast=174
v4l2-ctl -c saturation=207
v4l2-ctl -c white_balance_temperature_auto=1
v4l2-ctl -c gain=107
v4l2-ctl -c power_line_frequency=1
v4l2-ctl -c sharpness=128
v4l2-ctl -c backlight_compensation=1
v4l2-ctl -c exposure_auto=3
v4l2-ctl -c exposure_auto_priority=1
v4l2-ctl -c focus_auto=1
v4l2-ctl -c pan_absolute=0
v4l2-ctl -c tilt_absolute=0
v4l2-ctl -c zoom_absolute=100
v4l2-ctl --list-ctrls-menus