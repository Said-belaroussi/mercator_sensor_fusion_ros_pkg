#!/usr/bin/env python3

from mercator_sensor_fusion_ros_pkg.arena_poses_publisher import ArenaPosesPublisherNode
from mercator_sensor_fusion_ros_pkg.util import config_utils


if __name__ == '__main__':
    ros_args, user_args = config_utils.parse_args()

    # debug(ros_args, user_args)

    node = ArenaPosesPublisherNode()