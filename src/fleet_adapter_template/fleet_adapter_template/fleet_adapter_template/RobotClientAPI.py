# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import math
import time


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        self.prefix = config_yaml['prefix']
        self.user = config_yaml['user']
        self.password = config_yaml['password']
        self.timeout = 5.0
        self.debug = config_yaml.get('debug', False)
        self.robot_states = {
            'AGV1': {
                'initial_position': [0.0, 0.0, 0.0],
                'position': [0.0, 0.0, 0.0],
                'map': 'L1',
                'battery_soc': 1.0,
                'command_completed': True,
                'target': None,
                'activity': None,
                'activity_label': None,
                'last_update_time': time.monotonic(),
                'max_speed': 0.25,
                'speed_limit': 0.0,
                'manual_hold': False,
            },
            'AGV2': {
                'initial_position': [0.0, 2.0, 0.0],
                'position': [0.0, 2.0, 0.0],
                'map': 'L1',
                'battery_soc': 1.0,
                'command_completed': True,
                'target': None,
                'activity': None,
                'activity_label': None,
                'last_update_time': time.monotonic(),
                'max_speed': 0.25,
                'speed_limit': 0.0,
                'manual_hold': False,
            },
            'AGV3': {
                'initial_position': [0.0, -1.0, 0.0],
                'position': [0.0, -1.0, 0.0],
                'map': 'L1',
                'battery_soc': 1.0,
                'command_completed': True,
                'target': None,
                'activity': None,
                'activity_label': None,
                'last_update_time': time.monotonic(),
                'max_speed': 0.25,
                'speed_limit': 0.0,
                'manual_hold': False,
            },
            'AGV4': {
                'initial_position': [0.0, 3.0, 0.0],
                'position': [0.0, 3.0, 0.0],
                'map': 'L1',
                'battery_soc': 1.0,
                'command_completed': True,
                'target': None,
                'activity': None,
                'activity_label': None,
                'last_update_time': time.monotonic(),
                'max_speed': 0.25,
                'speed_limit': 0.0,
                'manual_hold': False,
            },
        }

    def _debug(self, message: str):
        if self.debug:
            print(f'[RobotAPI] {message}', flush=True)

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        self._debug(f'Checking connection to {self.prefix}')
        return True

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ):
        ''' Request the robot to localize on target map. This
            function should return True if the robot has accepted the
            request, else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return False
        if pose is None or len(pose) < 3:
            return False
        if not map_name:
            return False

        self.robot_states[robot_name]['position'] = [
            pose[0], pose[1], pose[2]
        ]
        self.robot_states[robot_name]['map'] = map_name
        self.robot_states[robot_name]['target'] = None
        self.robot_states[robot_name]['command_completed'] = True
        self.robot_states[robot_name]['last_update_time'] = time.monotonic()
        return True

    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return False
        if pose is None or len(pose) < 3:
            return False
        if not map_name:
            return False

        destination = [pose[0], pose[1], pose[2]]
        self._debug(
            f'Accepted navigate command for {robot_name}: '
            f'target={destination}, map={map_name}, '
            f'speed_limit={speed_limit}'
        )

        self.robot_states[robot_name]['target'] = destination
        self.robot_states[robot_name]['activity'] = None
        self.robot_states[robot_name]['activity_label'] = None
        self.robot_states[robot_name]['map'] = map_name
        self.robot_states[robot_name]['command_completed'] = False
        self.robot_states[robot_name]['last_update_time'] = time.monotonic()
        if speed_limit is not None and speed_limit > 0.0:
            self.robot_states[robot_name]['speed_limit'] = speed_limit
        else:
            self.robot_states[robot_name]['speed_limit'] = 0.0
        return True

    def pause(self, robot_name: str):
        '''Hold the fake robot in place without forgetting its target.'''
        if robot_name not in self.robot_states:
            return False

        self._debug(f'Accepted tutorial pause command for {robot_name}')
        self.robot_states[robot_name]['manual_hold'] = True
        self.robot_states[robot_name]['command_completed'] = False
        self.robot_states[robot_name]['last_update_time'] = time.monotonic()
        return True

    def resume(self, robot_name: str):
        '''Let the fake robot continue moving toward its current target.'''
        if robot_name not in self.robot_states:
            return False

        self._debug(f'Accepted tutorial resume command for {robot_name}')
        self.robot_states[robot_name]['manual_hold'] = False
        self.robot_states[robot_name]['last_update_time'] = time.monotonic()
        return True

    def reset(self, robot_name: str):
        '''Reset the fake robot to its initial charger pose.'''
        if robot_name not in self.robot_states:
            return False

        self._debug(f'Accepted tutorial reset command for {robot_name}')
        state = self.robot_states[robot_name]
        state['position'] = list(state['initial_position'])
        state['map'] = 'L1'
        state['target'] = None
        state['activity'] = None
        state['activity_label'] = None
        state['manual_hold'] = False
        state['command_completed'] = True
        state['last_update_time'] = time.monotonic()
        return True

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return False
        if not activity:
            return False

        self._debug(
            f'Accepted activity command for {robot_name}: '
            f'activity={activity}, label={label}'
        )

        self.robot_states[robot_name]['activity'] = activity
        self.robot_states[robot_name]['activity_label'] = label
        self.robot_states[robot_name]['target'] = None
        self.robot_states[robot_name]['command_completed'] = False

        # Tutorial behavior: pretend the activity finishes immediately.
        self.robot_states[robot_name]['command_completed'] = True
        return True

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return False

        self._debug(f'Accepted stop command for {robot_name}')

        self.robot_states[robot_name]['target'] = None
        self.robot_states[robot_name]['activity'] = None
        self.robot_states[robot_name]['activity_label'] = None
        self.robot_states[robot_name]['manual_hold'] = False
        self.robot_states[robot_name]['command_completed'] = True
        self.robot_states[robot_name]['last_update_time'] = time.monotonic()
        return True

    def _move_towards_target(self, robot_name: str):
        state = self.robot_states[robot_name]
        target = state['target']

        now = time.monotonic()
        dt = now - state['last_update_time']
        state['last_update_time'] = now

        if target is None or state['command_completed']:
            return

        if state['manual_hold']:
            return

        position = state['position']
        dx = target[0] - position[0]
        dy = target[1] - position[1]
        distance = math.hypot(dx, dy)

        if distance < 0.02:
            state['position'] = target
            state['target'] = None
            state['command_completed'] = True
            return

        speed = state['speed_limit'] or state['max_speed']
        step = min(speed * dt, distance)
        position[0] += step * dx / distance
        position[1] += step * dy / distance
        position[2] = math.atan2(dy, dx)

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return None
        self._move_towards_target(robot_name)
        return list(self.robot_states[robot_name]['position'])

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return None
        return self.robot_states[robot_name]['battery_soc']

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return None
        return self.robot_states[robot_name]['map']

    def is_command_completed(self, robot_name: str):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        if robot_name not in self.robot_states:
            return False
        return self.robot_states[robot_name]['command_completed']

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
