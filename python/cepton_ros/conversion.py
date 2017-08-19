from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import numpy

import geometry_msgs.msg
import rospy
import sensor_msgs.msg
import sensor_msgs.point_cloud2

import cepton_sdk


def get_point_field_dtype(field, is_bigendian):
    fmt, _ = sensor_msgs.point_cloud2._DATATYPES[field.datatype]
    byteorder = ">" if is_bigendian else "<"
    fmt = byteorder + fmt
    return numpy.dtype(fmt)


def get_point_step(fields):
    point_step = 0
    for field in fields:
        _, itemsize = sensor_msgs.point_cloud2._DATATYPES[field.datatype]
        point_step_tmp = field.offset + field.count * itemsize
        point_step = max(point_step, point_step_tmp)
    return point_step


def unpack_point_cloud_2(cloud):
    n_points = cloud.width * cloud.height

    data_bytes = \
        numpy.fromstring(cloud.data, dtype=numpy.uint8)
    data_bytes = data_bytes.reshape([n_points, cloud.point_step])
    data = {}
    for field in cloud.fields:
        dtype = get_point_field_dtype(field, is_bigendian=cloud.is_bigendian)
        field_size = field.count * dtype.itemsize

        data_bytes_tmp = \
            data_bytes[:, field.offset:field.offset + field_size].flatten()
        data[field.name] = \
            numpy.fromstring(data_bytes_tmp.tostring(), dtype=dtype)

    return data


def pack_point_cloud_2(header, fields, data):
    n_points = len(list(data.values())[0])
    point_step = get_point_step(fields)

    data_bytes = numpy.zeros([n_points, point_step], dtype=numpy.uint8)
    for field in fields:
        if field.name not in data:
            continue

        dtype = get_point_field_dtype(field, is_bigendian=False)
        field_size = field.count * dtype.itemsize

        data_tmp = data[field.name].astype(dtype)
        data_bytes_tmp = \
            numpy.fromstring(data_tmp.tostring(), dtype=numpy.uint8)
        data_bytes[:, field.offset:field.offset + field_size] = \
            data_bytes_tmp.reshape([n_points, dtype.itemsize])

    cloud = \
        sensor_msgs.msg.PointCloud2(
            header=header, height=1, width=n_points, is_dense=False,
            is_bigendian=False, fields=fields,
            point_step=point_step, row_step=n_points * point_step,
            data=data_bytes.flatten().tostring())
    return cloud
