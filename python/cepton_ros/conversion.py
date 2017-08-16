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


def point_cloud_2_to_cepton_image_points(cloud):
    n_points = cloud.width * cloud.height

    data = unpack_point_cloud_2(cloud)

    image_points = cepton_sdk.Points(n_points)
    image_points.timestamps[:] = 1e-6 * data["timestamp"]
    image_points.positions[:, 0] = data["image_x"]
    image_points.positions[:, 1] = data["image_z"]
    image_points.distances = data["distance"]
    image_points.intensities = data["intensity"]
    return image_points


def cepton_image_points_to_point_cloud_2(header, fields, image_points):
    data = {
        "timestamp": 1e6 * image_points.timestamps,
        "image_x": image_points.positions[:, 0],
        "image_z": image_points.positions[:, 1],
        "distance": image_points.distances,
        "intensity": points.intensities,
    }
    cloud = pack_point_cloud_2(header, fields, data)
    return cloud


def point_cloud_2_to_cepton_points(cloud):
    n_points = cloud.width * cloud.height

    data = unpack_point_cloud_2(cloud)

    points = cepton_sdk.Points(n_points)
    points.timestamps[:] = 1e-6 * data["timestamp"]
    points.positions[:, 0] = data["x"]
    points.positions[:, 1] = data["y"]
    points.positions[:, 2] = data["z"]
    points.intensities = data["intensity"]
    return points


def cepton_points_to_point_cloud_2(header, fields, points):
    data = {
        "timestamp": 1e6 * points.timestamps,
        "x": points.positions[:, 0],
        "y": points.positions[:, 1],
        "z": points.positions[:, 2],
        "intensity": points.intensities,
    }
    cloud = pack_point_cloud_2(header, fields, data)
    return cloud
