from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import cepton_ros.conversion
import cepton_sdk


def point_cloud_2_to_cepton_image_points(cloud):
    n_points = cloud.width * cloud.height

    data = cepton_ros.conversion.unpack_point_cloud_2(cloud)

    image_points = cepton_sdk.ImagePoints(n_points)
    image_points.timestamps_usec[:] = data["timestamps"]
    image_points.positions[:, 0] = data["image_x"]
    image_points.positions[:, 1] = data["image_z"]
    image_points.distances = data["distance"]
    image_points.intensities = data["intensity"]
    image_points.return_types = data["return_type"]
    return image_points


def cepton_image_points_to_point_cloud_2(header, fields, image_points):
    data = {
        "timestamp": 1e6 * image_points.timestamps_usec,
        "image_x": image_points.positions[:, 0],
        "image_z": image_points.positions[:, 1],
        "distance": image_points.distances,
        "intensity": image_points.intensities,
        "return_type": image_points.return_types,
    }
    cloud = cepton_ros.conversion.pack_point_cloud_2(header, fields, data)
    return cloud


def point_cloud_2_to_cepton_points(cloud):
    n_points = cloud.width * cloud.height

    data = cepton_ros.conversion.unpack_point_cloud_2(cloud)

    points = cepton_sdk.Points(n_points)
    points.timestamps_usec[:] = data["timestamp"]
    points.positions[:, 0] = data["x"]
    points.positions[:, 1] = data["y"]
    points.positions[:, 2] = data["z"]
    points.intensities = data["intensity"]
    points.return_types = data["return_types"]
    return points


def cepton_points_to_point_cloud_2(header, fields, points):
    data = {
        "timestamp": points.timestamps_usec,
        "x": points.positions[:, 0],
        "y": points.positions[:, 1],
        "z": points.positions[:, 2],
        "intensity": points.intensities,
        "return_type": points.return_types,
    }
    cloud = cepton_ros.pack_point_cloud_2(header, fields, data)
    return cloud
