from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import cepton_sdk


class C_SensorPoint(Structure):
    _fields_ = cepton_sdk.c.C_SensorImagePoint + [
        ("reserved", 4 * c_uint8),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]


def ros_to_cepton_points(cloud):
    n_points = cloud.width * cloud.height
    c_points = cepton_sdk.common.c.convert_bytes_to_ndarray(
        numpy.fromstring(cloud.data, dtype=numpy.uint8), C_SensorPoint)
    return cepton_sdk.Points.from_c(n_points, c_points)
