from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Frame

from compas_rcf.ur.helpers import format_urscript_cmd
from compas_rcf.ur.helpers import format_pose
from compas_rcf.ur.helpers import format_joint_positions

# Motion


@format_urscript_cmd
def movel(frame_to, accel=1.2, vel=.25, time=0, zone=0) -> str:
    pose = format_pose(frame_to)
    return "movel({:s}, a={:.2f}, v={:2f}, t={:2f} r={:2f})".format(pose, accel, vel, time, zone)


@format_urscript_cmd
def movej(joint_positions, accel=1.4, vel=1.05, time=0, zone=0):
    """
    Function that returns UR script for linear movement in joint space.

    Args:
        joints: A list of 6 joint angles (double).
        accel: tool accel in m/s^2
        accel: tool accel in m/s^2
        vel: tool speed in m/s

    Returns:
        script: UR script
    """
    # TODO: Test
    # TODO: Check acceleration and velocity are below set limit
    _joint_positions = format_joint_positions(joint_positions)

    return "movej({:s}, a={:.2f}, v={:.2f}), t={:.2f}, r={:.2f}".format(_joint_positions, accel, vel, time, zone)


# Utility


@format_urscript_cmd
def set_TCP(tcp_frame: Frame) -> str:
    pose = format_pose(tcp_frame)
    return "set_tcp({:s})".format(pose)


@format_urscript_cmd
def textmsg(string: str) -> str:
    return "textmsg(\"" + string + "\")"


@format_urscript_cmd
def popup(string: str) -> str:
    # Popup title not implemented, neither is error or warning flags
    return "popup(\"{}\")".format(string)


@format_urscript_cmd
def sleep(seconds: float) -> str:
    """
    Function that returns UR script for sleep()

    Args:
        time: float.in s

    Returns:
        script: UR script
    """
    return "sleep({})\n".format(seconds)


# Communication


@format_urscript_cmd
def socket_open(server_address: str, server_port: int) -> str:
    return "socket_open(\"{}\", {:d})".format(server_address, server_port)


@format_urscript_cmd
def socket_send_string(text: str) -> str:
    return "socket_send_string(\"" + text + "\")"


@format_urscript_cmd
def socket_close() -> str:
    return "socket_close()"


# IO


@format_urscript_cmd
def set_DO(pin: int, state: bool) -> str:
    # deprecation warning in UR manual
    # return "set_digital_out({:d}, {})".format(pin, state)
    return set_standard_digital_out(pin, state)


@format_urscript_cmd
def set_standard_digital_out(pin: int, state: bool):
    """Set standard digital output signal level

    Parameters
    ----------
    pin : int, 0 to 7
        Pin number
    state: bool

    Returns
    -------
    str
    """
    # TODO: Test
    return "set_standard_digital_out({}, {})".format(id, str(state))


@format_urscript_cmd
def set_standard_analog_out(pin: int, signal_level: float):
    """Set standard analog output level

    Parameters
    ----------
    pin : int, 0 to 1
        Pin number
    signal_level : float, 0 to 1
        Relative signal level

    Returns
    -------
    str
    """
    # TODO: Test

    return "set_standard_analog_out({}, {:.3})".format(id, signal_level)
