from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import cepton_ros.conversion


def point_cloud_2_to_cepton_image_points(cloud):
    n_points = cloud.width * cloud.height

    data = cepton_ros.conversion.unpack_point_cloud_2(cloud)

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
    cloud = cepton_ros.conversion.pack_point_cloud_2(header, fields, data)
    return cloud

def point_cloud_2_to_cepton_points(cloud):
    n_points = cloud.width * cloud.height

    data = cepton_ros.conversion.unpack_point_cloud_2(cloud)

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
    cloud = cepton_ros.pack_point_cloud_2(header, fields, data)
    return cloud
