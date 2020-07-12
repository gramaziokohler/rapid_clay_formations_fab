"""Arbitrary points method for robot relocalization.

Code adapted from source code by Selen Ercan et al at Gramazio Kohler Research,
ETH Zürich (2019).

Original code:
https://github.com/gramaziokohler/IF_jamming/blob/master/if_jamming/localization/arbitrary_point_localization.py

Ercan, Selen, Sandro Meier, Fabio Gramazio, and Matthias Kohler. 2019.
“Automated Localization of a Mobile Construction Robot with an External
Measurement Device.” In Proceedings of the 36th International Symposium on
Automation and Robotics in Construction (ISARC 2019), 929–36. International
Association on Automation and Robotics in Construction.
https://doi.org/10.3929/ethz-b-000328442.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import tempfile
from functools import reduce

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


def _objective_function(x, *args):
    """Objective function for the optimization problem.

    Parameters
    ----------
    x
        The optimization variable (9x1)
    args : :obj:`tuple`
        The localization points and the measurements as a tuple of two
        dimensional arrays where each row is one point. The columns are the X,
        Y and Z coordinates.

    Returns
    -------
    :obj:`float`
        The cost for the given optimization variable values.
    """
    localization_points = args[0]
    measurements = args[1]

    origin = np.array(x[0:3])
    x_vec = np.array(x[3:6])
    y_vec = np.array(x[6:9])
    z_vec = np.cross(x_vec, y_vec)

    cost = 0
    for point, measurement in zip(localization_points, measurements):
        # Calculate the deviation from the measurement using the given
        # coordinate system (optimization variable) and add the square of it to
        # the cost.
        deviation = np.power(
            origin
            + point[0] * x_vec
            + point[1] * y_vec
            + point[2] * z_vec
            - measurement,
            2,
        )
        cost += sum(deviation)

    return cost


def _nonlinear_constraints(x, *args):
    """Constraints for the optimization problem.

    Parameters
    ----------
    x
        The optimization variable (9x1).

    Returns
    -------
    :obj:`list` of :obj:`float`
        An array that contains the values when the constraints are evaluated at
        `x`.
    """
    return [
        # x and y need to be orthogonal (i.e. scalar product = 0)
        x[3] * x[6] + x[4] * x[7] + x[5] * x[8],
        x[3] ** 2 + x[4] ** 2 + x[5] ** 2 - 1,  # |x| = 1
        x[6] ** 2 + x[7] ** 2 + x[8] ** 2 - 1,  # |y| = 1
    ]


def _nonlinear_jacobian(x):
    """Jacobian for the constraints.

    Parameters
    ----------
    x
        The optimization variable (9x1).

    Returns
    -------
        The jacobian of the nonlinear constraints.
    """
    return [
        [0, 0, 0, x[6], x[7], x[8], x[3], x[4], x[5]],
        [0, 0, 0, 2 * x[3], 2 * x[4], 2 * x[5], 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 2 * x[6], 2 * x[7], 2 * x[8]],
    ]


def calculate_rcs_origin(localization_points, measurements):
    """Calculate the RCS origin frame.

    Finding the origin is formulated as an optimization problem where we want
    to find the origin and two orthonormal vectors defining the coordinate
    system. At the same time, the position of the localization points in this
    new coordinate system should match the measurements as close as possible.
    Therefore the deviation from the measurements is used as the cost function.
    The only constraints are that the x and y vector need to have length 1 and
    be orthogonal to each other.  The optimization variable is a vector with 9
    entries: X = [o, x, y] where o is the origin of the coordinate system and
    x, y the vectors spanning the x-y-plane. Each of them is a 3 dimensional
    vector.  **Important**: Ensure that the order of localization_points and
    measurements is identical. I.e. the i-th entry in measurements is the
    measurement of the i-th localization point.

    Parameters
    ----------
    localization_points : :obj:`tuple` of :obj:`float`
        The points where the robot endeffector was positioned to take
        measurements. These points are in the RCS.
    measurements : :obj:`tuple` of :obj:`float`
        The measurements taken in the world coordinate system (WCS) with the
        total station. These are the coordinates of the localization_points in
        the WCS.

    Returns
    -------
    :obj:`tuple` of :obj:`tuple` of :obj:`float`
        A tuple of 3 vectors (lists with 3 elements) where the first represents
        the origin of the RCS, the second is the direction of the x axis and
        the third the direction of the y axis. The x and y axis are vectors
        with length 1.
    """
    # Setup the constraints
    constraints = {"type": "eq", "fun": _nonlinear_constraints}

    print(localization_points)
    print(measurements)

    results = []
    slices = 4
    for i in range(slices):
        radians = np.deg2rad(360.0 / float(slices) * i)
        c, s = np.cos(radians), np.sin(radians)
        rotation = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))

        x = rotation.dot(np.array([1, 0, 0]))
        y = rotation.dot(np.array([0, 1, 0]))

        # We use the standard coordinate system as an initial guess.
        x0 = np.array(np.concatenate(([0, 0, 0], x, y)))
        res = minimize(
            _objective_function,
            x0,
            args=(localization_points, measurements),
            constraints=constraints,
            # jac=_nonlinear_jacobian,
            options={"disp": True},
        )
        results.append(res)

    # plot_results(localization_points, measurements, results)

    # Pick the result with the lowest objective value
    print(results)
    result = reduce((lambda x, y: x if x.fun < y.fun else y), results)

    origin = result.x[0:3].tolist()
    x_vec = result.x[3:6].tolist()
    y_vec = result.x[6:9].tolist()

    return [origin, x_vec, y_vec]


def validate_arguments(args):
    """Leftover from pickle workflow."""
    # Check if the input file exists
    input_file = args.input_file
    if not os.path.isfile(input_file):
        raise Exception("Input file does not exist or cannot be accessed")

    # Load it and check if all necessary keys exist.
    with open(args.input_file, mode="rb") as f:
        print(f)
        input_ = json.load(f)
        # input = pickle.load(f)
    keys = ["localization_points", "measurements"]
    for key in keys:
        if key not in input_:
            raise Exception("Required key missing: {}".format(key))

    print(input_)
    return input_[keys[0]], input_[keys[1]]


def plot_results(localization_points, measurements, results):
    """Create plots to visualize multiple consecutive results from a solver."""
    for i, res in zip(range(len(results)), results):
        directory = tempfile.mkdtemp("", "localization_{}".format(i))
        _plot_result(localization_points, measurements, res, directory)
        print("Saving plots to {}".format(directory))

    # Create a plot summarizing the the different runs
    objective_values = [result.fun for result in results]
    summary_file = tempfile.mktemp("_summary.png", "localization_")
    plt.figure()
    plt.plot(range(len(objective_values)), objective_values, "ro")
    plt.ylabel("Objective value")
    plt.xlabel("Run")
    plt.title("Objective value for different x_0")
    plt.savefig(summary_file)
    plt.show()


def _plot_result(localization_points, measurements, result, folder):
    """Create some plots that illustrate the result.

    Parameters
    ----------
    localization_points
    measurements
    result
        The result from the solver
    folder
        The folder in which the plots should be stored.

    Returns
    -------
    :obj:`str`
        The path at which the plots were stored.
    """
    # First we plot just the localization points
    _localization_point_plot(measurements, localization_points, result, folder)


def _localization_point_plot(measurements, localization_points, result, folder):
    origin = result.x[0:3]
    x_vec = result.x[3:6]
    y_vec = result.x[6:9]
    z_vec = np.cross(x_vec, y_vec)

    x_axis = origin + 1000 * x_vec
    y_axis = origin + 1000 * y_vec
    # z_axis = origin + 1000 * z_vec

    # Calculate the localization points in the new coordinate system
    transformed_points = []
    for point in localization_points:
        transformed_points.append(
            origin + point[0] * x_vec + point[1] * y_vec + point[2] * z_vec
        )
    transformed_points = np.array(transformed_points)

    plt.figure()
    plt.plot(measurements.T[0], measurements.T[1], "bo")
    plt.plot(transformed_points.T[0], measurements.T[1], "rx")
    plt.plot(origin[0], origin[1], "bx")
    plt.plot(x_axis[0], x_axis[1], "rx")
    plt.plot(y_axis[0], y_axis[1], "gx")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("X-Y projection")
    plt.savefig(os.path.join(folder, "rcs_matching_xy.png"))

    plt.figure()
    plt.plot(measurements.T[0], measurements.T[2], "bo")
    plt.plot(transformed_points.T[0], measurements.T[2], "rx")
    plt.plot(origin[0], origin[2], "bx")
    plt.plot(x_axis[0], x_axis[2], "rx")
    plt.plot(y_axis[0], y_axis[2], "gx")
    plt.xlabel("x")
    plt.ylabel("z")
    plt.title("X-Z projection")
    plt.savefig(os.path.join(folder, "rcs_matching_xz.png"))

    plt.figure()
    plt.plot(measurements.T[1], measurements.T[2], "bo")
    plt.plot(transformed_points.T[1], measurements.T[2], "rx")
    plt.plot(origin[1], origin[2], "bx")
    plt.plot(x_axis[1], x_axis[2], "rx")
    plt.plot(y_axis[1], y_axis[2], "gx")
    plt.xlabel("y")
    plt.ylabel("z")
    plt.title("Y-Z projection")
    plt.savefig(os.path.join(folder, "rcs_matching_yz.png"))


if __name__ == "__main__":
    """Leftover from pickle workflow."""
    import argparse
    import json

    parser = argparse.ArgumentParser(description="RCS Optimizer")
    parser.add_argument("input_file", type=str)
    parser.add_argument("output_file", type=str)
    args = parser.parse_args()
    print(args)
    localization_points, measurements = validate_arguments(args)
    localization_points = np.array(localization_points)
    measurements = np.array(measurements)

    rcs = calculate_rcs_origin(localization_points, measurements)

    origin = rcs.x[0:3].tolist()
    x_vec = rcs.x[3:6].tolist()
    y_vec = rcs.x[6:9].tolist()

    output = {"rcs": [origin, x_vec, y_vec]}
    with open(args.output_file, "w") as f:
        json.dump(output, f)
    print("Wrote output to: {}".format(args.output_file))
