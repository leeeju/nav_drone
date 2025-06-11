
import sys
from typing import Dict, Text

import yaml


class ParseMultidronePose:
    """Parsing argument using sys module."""

    def __init__(self, target_argument: Text):
        """
        Parse arguments for multi-drone's pose.

        for example,
        `ros2 launch nav2_bringup bringup_multidrone_launch.py
            drones:="drone1={x: 1.0, y: 1.0, yaw: 0.0};
                     drone2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"`

        `target_argument` shall be 'drones'.
        Then, this will parse a string value for `drones` argument.

        Each drone name which is corresponding to namespace and pose of it will be separted by `;`.
        The pose consists of x, y and yaw with YAML format.

        :param: target argument name to parse
        """
        self.__args: Text = self.__parse_argument(target_argument)

    def __parse_argument(self, target_argument: Text) -> Text:
        """Get value of target argument."""
        if len(sys.argv) > 4:
            argv = sys.argv[4:]
            for arg in argv:
                if arg.startswith(target_argument + ':='):
                    return arg.replace(target_argument + ':=', '')
        return ''

    def value(self) -> Dict:
        """Get value of target argument."""
        args = self.__args
        parsed_args = [] if len(args) == 0 else args.split(';')
        multidrones = {}
        for arg in parsed_args:
            key_val = arg.strip().split('=')
            if len(key_val) != 2:
                continue
            key = key_val[0].strip()
            val = key_val[1].strip()
            drone_pose = yaml.safe_load(val)
            if 'x' not in drone_pose:
                drone_pose['x'] = 0.0
            if 'y' not in drone_pose:
                drone_pose['y'] = 0.0
            if 'z' not in drone_pose:
                drone_pose['z'] = 0.0
            if 'roll' not in drone_pose:
                drone_pose['roll'] = 0.0
            if 'pitch' not in drone_pose:
                drone_pose['pitch'] = 0.0
            if 'yaw' not in drone_pose:
                drone_pose['yaw'] = 0.0
            multidrones[key] = drone_pose
        return multidrones
