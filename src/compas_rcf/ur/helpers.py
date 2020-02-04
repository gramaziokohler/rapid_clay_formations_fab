from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import re
from functools import wraps

import compas.geometry as cg
import Rhino.Geometry as rg
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Rotation
from compas.geometry import Transformation
from compas_fab.robots.configuration import Configuration
from compas_fab.robots.ur5 import Robot

from compas_rcf.ur.urscript_wrapper import set_DO
from compas_rcf.ur.urscript_wrapper import set_standard_analog_out
from compas_rcf.ur.urscript_wrapper import textmsg
from compas_rcf.utils.compas_to_rhino import cgframe_to_rgplane
from compas_rcf.utils.compas_to_rhino import matrix_to_rgtransform
from compas_rcf.utils.rhino_to_compas import rgtransform_to_matrix
from compas_rcf.utils.rhino_to_compas import rgplane_to_cgframe

# Programs


def turn_off_outputs_program():
    script = ""
    script += "def program():\n"
    script += textmsg("Turning of all DOs and AOs.")
    for i in range(7):
        script += set_DO(i, False)
    for i in range(1):
        script += set_standard_analog_out(i, 0)
    script += "end\n"
    script += "program()\n"
    return script


# Geometry


def axis_angle_vector_from_plane_to_plane(plane_to, plane_from=rg.Plane.WorldXY):
    T = rg.Transform.PlaneToPlane(plane_from, plane_to)
    M = rgtransform_to_matrix(T)
    return cg.axis_angle_vector_from_matrix(M)


# UR script helpers


def format_joint_positions(joint_values):
    jpos_fmt = "[" + ", ".join(["{:.4f}"] * 6) + "]"
    return jpos_fmt.format(*joint_values)


def format_pose(frame_like):
    if isinstance(frame_like, rg.Plane):
        frame = rgplane_to_cgframe(frame_like)
    elif isinstance(frame_like, cg.Frame):
        frame = frame_like
    else:
        raise TypeError

    pose_data = [c / 1000. for c in frame.origin.data] + frame.axis_angle_vector()
    pose_fmt = "p[" + ", ".join(["{:.4f}"] * 6) + "]"
    return pose_fmt.format(*pose_data)


def format_urscript_cmd(func):
    @wraps
    def wrapper(*arg):
        return "\t" + func(*arg) + "\n"

    return wrapper


def visualize_ur_script(script):
    M = [[-1000,    0,    0,    0], # noqa E201
         [    0, 1000,    0,    0], # noqa E201
         [    0,    0, 1000,    0], # noqa E201
         [    0,    0,    0,    1]] # noqa E201 # yapf: disable
    rgT = matrix_to_rgtransform(M)
    cgT = Transformation.from_matrix(M)

    robot = Robot()

    viz_planes = []

    movel_matcher = re.compile(r'^\s*move([lj]).+((-?\d+\.\d+,?\s?){6}).*$')
    for line in script.splitlines():
        mo = re.search(movel_matcher, line)
        if mo:
            if mo.group(1) == 'l':  # movel
                ptX, ptY, ptZ, rX, rY, rZ = mo.group(2).split(',')

                pt = Point(float(ptX), float(ptY), float(ptZ))
                pt.transform(cgT)
                frame = Frame(pt, [1, 0, 0], [0, 1, 0])

                R = Rotation.from_axis_angle_vector([float(rX), float(rY), float(rZ)], pt)
                T = matrix_to_rgtransform(R)

                plane = cgframe_to_rgplane(frame)
                plane.Transform(T)

                viz_planes.append(plane)
            else:  # movej
                joint_values = mo.group(2).split(',')
                configuration = Configuration.from_revolute_values([float(d) for d in joint_values])
                frame = robot.forward_kinematics(configuration)

                plane = cgframe_to_rgplane(frame)
                plane.Transform(rgT)

                viz_planes.append(plane)

    return viz_planes
