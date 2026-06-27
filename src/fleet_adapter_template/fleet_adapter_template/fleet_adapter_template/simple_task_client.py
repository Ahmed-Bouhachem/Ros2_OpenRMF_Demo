# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest, ApiResponse, TaskType
from rmf_task_msgs.srv import CancelTask, GetDispatchStates, SubmitTask
from std_msgs.msg import String
from std_srvs.srv import Trigger


ROUTES = {
    'route_a': ('pickup', 'dropoff'),
    'route_b': ('pickup_B', 'dropoff_B'),
    'route_c': ('inspection', 'storage'),
    'route_d': ('storage', 'dropoff_B'),
    'route_e': ('receiving', 'shipping'),
    'route_f': ('maintenance', 'quality_check'),
    'route_g': ('pallet_rack_A', 'staging'),
    'route_h': ('returns', 'packing'),
    'route_i': ('battery_swap', 'pallet_rack_B'),
    'route_j': ('receiving', 'returns'),
    'route_k': ('pickup_B', 'battery_swap'),
    'route_l': ('pallet_rack_B', 'dropoff'),
}

CONFLICT_ROUTES = [
    ('conflict_a', *ROUTES['route_a']),
    ('conflict_b', *ROUTES['route_b']),
]

ALL_ROUTES = [
    (route_name, *route)
    for route_name, route in ROUTES.items()
]

SHOWCASE_ROUTES = [
    ('upper_pickup', *ROUTES['route_a']),
    ('lower_pickup', *ROUTES['route_b']),
    ('inspection_run', *ROUTES['route_c']),
    ('storage_return', *ROUTES['route_d']),
    ('receiving_to_shipping', *ROUTES['route_e']),
    ('maintenance_check', *ROUTES['route_f']),
    ('rack_to_staging', *ROUTES['route_g']),
    ('returns_to_packing', *ROUTES['route_h']),
    ('battery_to_rack', *ROUTES['route_i']),
    ('receiving_to_returns', *ROUTES['route_j']),
    ('cross_map_battery', *ROUTES['route_k']),
    ('pallet_dropoff', *ROUTES['route_l']),
]


MENU = '''
Simple RMF task menu

1. Send route A        pickup -> dropoff
2. Send route B        pickup_B -> dropoff_B
3. Send route C        inspection -> storage
4. Send route D        storage -> dropoff_B
5. Send route E        receiving -> shipping
6. Send route F        maintenance -> quality_check
7. Send route G        pallet_rack_A -> staging
8. Send route H        returns -> packing
9. Two-task conflict demo
10. Large map decision demo
11. Show robot status
12. Custom loop
13. List dispatches
14. Cancel task with task API
15. Cancel task with dispatcher service
16. Pause fake robots
17. Resume fake robots
18. Reset fake robots
19. Run video showcase
q. Exit
'''


class SimpleTaskClient(Node):
    def __init__(self):
        super().__init__('simple_task_client')
        self.submit_task = self.create_client(SubmitTask, '/submit_task')
        self.cancel_task_client = self.create_client(CancelTask, '/cancel_task')
        self.get_dispatches_client = self.create_client(
            GetDispatchStates,
            '/get_dispatches',
        )
        self.pause_robots_client = self.create_client(
            Trigger,
            '/simple_pause_robots',
        )
        self.resume_robots_client = self.create_client(
            Trigger,
            '/simple_resume_robots',
        )
        self.reset_robots_client = self.create_client(
            Trigger,
            '/simple_reset_robots',
        )
        self.task_intent_pub = self.create_publisher(
            String,
            '/simple_task_client/task_intents',
            10,
        )
        task_api_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.task_api_request_pub = self.create_publisher(
            ApiRequest,
            '/task_api_requests',
            task_api_qos,
        )
        self.task_api_responses = {}
        self.create_subscription(
            ApiResponse,
            '/task_api_responses',
            self.handle_task_api_response,
            task_api_qos,
        )

    def handle_task_api_response(self, msg):
        self.task_api_responses.setdefault(msg.request_id, []).append(msg)

    def trigger_simple_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {service_name} is not available')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'{service_name} call failed: {future.exception()}'
            )
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(response.message)
            return True

        self.get_logger().error(response.message)
        return False

    def pause_all_robots(self):
        return self.trigger_simple_service(
            self.pause_robots_client,
            '/simple_pause_robots',
        )

    def resume_all_robots(self):
        return self.trigger_simple_service(
            self.resume_robots_client,
            '/simple_resume_robots',
        )

    def reset_all_robots(self):
        return self.trigger_simple_service(
            self.reset_robots_client,
            '/simple_reset_robots',
        )

    def submit_loop_task(
        self,
        task_id,
        start_name,
        finish_name,
        requester='tutorial',
        robot_type='',
        num_loops=1,
    ):
        if not self.submit_task.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /submit_task is not available')
            return False

        request = SubmitTask.Request()
        request.requester = requester
        request.description.start_time.sec = 0
        request.description.start_time.nanosec = 0
        request.description.priority.value = 0
        request.description.task_type.type = TaskType.TYPE_LOOP
        request.description.loop.task_id = task_id
        request.description.loop.robot_type = robot_type
        request.description.loop.num_loops = num_loops
        request.description.loop.start_name = start_name
        request.description.loop.finish_name = finish_name

        self.get_logger().info(
            f'Submitting loop task [{task_id}]: '
            f'{start_name} -> {finish_name}'
        )

        future = self.submit_task.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'/submit_task call failed: {future.exception()}'
            )
            return False

        response = future.result()
        if response.success:
            self.publish_task_intent(
                response.task_id,
                task_id,
                start_name,
                finish_name,
                num_loops,
            )
            self.get_logger().info(
                f'Accepted by RMF as [{response.task_id}]'
            )
            return True

        self.get_logger().error(
            f'RMF rejected task [{task_id}]: {response.message}'
        )
        return False

    def cancel_task(self, task_id, requester='tutorial'):
        if not task_id:
            self.get_logger().error('A task ID is required for cancel')
            return False

        if not self.cancel_task_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /cancel_task is not available')
            return False

        request = CancelTask.Request()
        request.requester = requester
        request.task_id = task_id

        self.get_logger().info(f'Requesting cancel for task [{task_id}]')
        future = self.cancel_task_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'/cancel_task call failed: {future.exception()}'
            )
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(f'Cancel accepted for [{task_id}]')
            return True

        message = response.message or (
            'Task may already be finished, canceled, or unknown to this '
            'dispatcher. Try "api_cancel" for already-dispatched tasks.'
        )
        self.get_logger().error(
            f'RMF rejected cancel for [{task_id}]: {message}'
        )
        return False

    def api_cancel_task(self, task_id, labels=None, timeout_sec=10.0):
        if not task_id:
            self.get_logger().error('A task ID is required for api_cancel')
            return False

        labels = labels or ['simple_task_client', 'tutorial_cancel']
        request_id = f'simple_task_client_{uuid.uuid4().hex}'
        request = ApiRequest()
        request.request_id = request_id
        request.json_msg = json.dumps({
            'type': 'cancel_task_request',
            'task_id': task_id,
            'labels': labels,
        })

        self.wait_for_task_api_subscription()
        self.get_logger().info(
            f'Requesting task API cancel for [{task_id}] '
            f'with request_id [{request_id}]'
        )

        for _ in range(3):
            self.task_api_request_pub.publish(request)
            rclpy.spin_once(self, timeout_sec=0.1)

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            responses = self.task_api_responses.get(request_id, [])
            while responses:
                response = responses.pop(0)
                if response.type == ApiResponse.TYPE_ACKNOWLEDGE:
                    self.get_logger().info(
                        f'Task API acknowledged cancel for [{task_id}]'
                    )
                    continue

                if response.type != ApiResponse.TYPE_RESPONDING:
                    self.get_logger().warn(
                        f'Ignoring task API response with type '
                        f'[{response.type}]'
                    )
                    continue

                return self.handle_task_api_result(task_id, response)

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f'Timed out waiting for task API cancel response for [{task_id}]'
        )
        return False

    def wait_for_task_api_subscription(self, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.task_api_request_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            'No subscribers currently visible on /task_api_requests. '
            'Publishing anyway; make sure the fleet adapter is running.'
        )
        return False

    def handle_task_api_result(self, task_id, response):
        try:
            result = json.loads(response.json_msg) if response.json_msg else {}
        except json.JSONDecodeError as err:
            self.get_logger().error(
                f'Task API returned bad JSON for [{task_id}]: {err}'
            )
            return False

        if result.get('success') is True:
            self.get_logger().info(
                f'Task API cancel accepted for [{task_id}]'
            )
            return True

        errors = result.get('errors') or []
        if errors:
            self.get_logger().error(
                f'Task API rejected cancel for [{task_id}]: '
                f'{self.format_api_errors(errors)}'
            )
        else:
            self.get_logger().error(
                f'Task API rejected cancel for [{task_id}]: {result}'
            )
        return False

    @staticmethod
    def format_api_errors(errors):
        messages = []
        for error in errors:
            if isinstance(error, dict):
                messages.append(
                    str(error.get('detail') or error.get('code') or error)
                )
            else:
                messages.append(str(error))
        return '; '.join(messages)

    def sleep_with_spin(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_showcase(self, requester='tutorial', robot_type='', num_loops=1):
        print('Showcase step 0: confirm the dispatcher is clean')
        states = self.get_dispatch_states()
        if states is None:
            return False
        if states.active:
            print(
                'The dispatcher still has active tasks. For a clean video, '
                'restart the demo first:'
            )
            print('  scripts/stop_rmf_demo.sh')
            print('  scripts/start_rmf_demo.sh')
            print('  scripts/rmf_task.sh showcase')
            print()
            self.print_dispatch_group('active', states.active)
            return False
        print('No active dispatches. Starting a clean showcase.')

        print()
        print('Showcase step 1: reset fake robots to charger poses')
        self.reset_all_robots()
        self.sleep_with_spin(1.0)
        self.print_status(timeout_sec=3.0)

        base_task_id = f'linkedin_showcase_{int(time.time())}'
        print()
        print('Showcase step 2: submit twelve tasks for four AGVs')
        ok = True
        for label, start_name, finish_name in SHOWCASE_ROUTES:
            ok = self.submit_loop_task(
                f'{base_task_id}_{label}',
                start_name,
                finish_name,
                requester=requester,
                robot_type=robot_type,
                num_loops=num_loops,
            ) and ok
            self.sleep_with_spin(0.3)

        print()
        print('Showcase step 3: RMF bidding, assignment, and queueing')
        self.sleep_with_spin(3.0)
        self.print_dispatches()

        print()
        print('Showcase step 4: watch RViz for route and traffic decisions')
        for _ in range(3):
            self.sleep_with_spin(4.0)
            self.print_status(timeout_sec=3.0)

        print()
        print('Showcase is running. Keep recording RViz, or use status/menu.')
        return ok

    def get_dispatch_states(self):
        if not self.get_dispatches_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /get_dispatches is not available')
            return None

        request = GetDispatchStates.Request()
        request.task_ids = []

        future = self.get_dispatches_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'/get_dispatches call failed: {future.exception()}'
            )
            return None

        response = future.result()
        if not response.success:
            self.get_logger().error('RMF rejected /get_dispatches request')
            return None

        return response.states

    def print_dispatches(self):
        states = self.get_dispatch_states()
        if states is None:
            return False

        self.print_dispatch_group('active', states.active)
        self.print_dispatch_group('finished', states.finished)
        return True

    def cancel_active_dispatches(self):
        states = self.get_dispatch_states()
        if states is None:
            return False

        active_dispatches = list(states.active)
        if not active_dispatches:
            print('No active dispatches to clear.')
            return True

        print(f'Canceling {len(active_dispatches)} active dispatch(es).')
        ok = True
        for dispatch in active_dispatches:
            ok = self.api_cancel_task(
                dispatch.task_id,
                labels=['simple_task_client', 'showcase_cleanup'],
                timeout_sec=3.0,
            ) and ok
            self.sleep_with_spin(0.2)

        return ok

    def print_dispatch_group(self, name, dispatches):
        print(f'{name}:')
        if not dispatches:
            print('  none')
            return

        for dispatch in dispatches:
            assignment = dispatch.assignment
            if assignment.is_assigned:
                assigned_to = (
                    f'{assignment.fleet_name}/{assignment.expected_robot_name}'
                )
            else:
                assigned_to = '-'
            errors = ', '.join(dispatch.errors) if dispatch.errors else '-'
            print(
                f'  {dispatch.task_id}: '
                f'status={self.dispatch_status_name(dispatch.status)}, '
                f'assigned={assigned_to}, '
                f'errors={errors}'
            )

    @staticmethod
    def dispatch_status_name(status):
        return {
            0: 'uninitialized',
            1: 'queued',
            2: 'selected',
            3: 'dispatched',
            4: 'failed_to_assign',
            5: 'canceled_in_flight',
        }.get(status, str(status))

    def publish_task_intent(
        self,
        rmf_task_id,
        client_task_id,
        start_name,
        finish_name,
        num_loops,
    ):
        msg = String()
        msg.data = json.dumps({
            'type': 'loop',
            'rmf_task_id': rmf_task_id,
            'client_task_id': client_task_id,
            'start_name': start_name,
            'finish_name': finish_name,
            'num_loops': num_loops,
        })
        for _ in range(3):
            self.task_intent_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def print_status(self, timeout_sec=5.0):
        latest_msg = None

        def handle_fleet_state(msg):
            nonlocal latest_msg
            latest_msg = msg

        subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            handle_fleet_state,
            10,
        )

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and latest_msg is None:
            if time.monotonic() > deadline:
                self.get_logger().error('Timed out waiting for /fleet_states')
                self.destroy_subscription(subscription)
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(subscription)
        for robot in latest_msg.robots:
            location = robot.location
            task_id = robot.task_id or '-'
            print(
                f'{robot.name}: '
                f'task={task_id}, '
                f'x={location.x:.2f}, '
                f'y={location.y:.2f}, '
                f'yaw={location.yaw:.2f}, '
                f'battery={robot.battery_percent:.1f}%, '
                f'mode={robot.mode.mode}'
            )

        return True

    def run_menu(self, requester='tutorial', robot_type='', num_loops=1):
        while rclpy.ok():
            print(MENU)
            choice = input('Choose: ').strip().lower()

            if choice in ('q', 'quit', 'exit'):
                print('Bye.')
                return True

            if choice == '1':
                self.submit_menu_route(
                    'route_a',
                    *ROUTES['route_a'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '2':
                self.submit_menu_route(
                    'route_b',
                    *ROUTES['route_b'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '3':
                self.submit_menu_route(
                    'route_c',
                    *ROUTES['route_c'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '4':
                self.submit_menu_route(
                    'route_d',
                    *ROUTES['route_d'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '5':
                self.submit_menu_route(
                    'route_e',
                    *ROUTES['route_e'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '6':
                self.submit_menu_route(
                    'route_f',
                    *ROUTES['route_f'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '7':
                self.submit_menu_route(
                    'route_g',
                    *ROUTES['route_g'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '8':
                self.submit_menu_route(
                    'route_h',
                    *ROUTES['route_h'],
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '9':
                print('Sending both routes through the shared traffic zone.')
                for label, start_name, finish_name in CONFLICT_ROUTES:
                    self.submit_menu_route(
                        label,
                        start_name,
                        finish_name,
                        requester=requester,
                        robot_type=robot_type,
                        num_loops=num_loops,
                    )
            elif choice == '10':
                print('Sending 12 routes so RMF can choose across 4 AGVs.')
                for label, start_name, finish_name in ALL_ROUTES:
                    self.submit_menu_route(
                        label,
                        start_name,
                        finish_name,
                        requester=requester,
                        robot_type=robot_type,
                        num_loops=num_loops,
                    )
            elif choice == '11':
                self.print_status()
            elif choice == '12':
                start_name = input('Start waypoint: ').strip()
                finish_name = input('Finish waypoint: ').strip()
                if not start_name or not finish_name:
                    print('Start and finish waypoint names are required.')
                    continue
                self.submit_menu_route(
                    'custom_loop',
                    start_name,
                    finish_name,
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            elif choice == '13':
                self.print_dispatches()
            elif choice == '14':
                self.print_dispatches()
                task_id = input('Task ID to cancel: ').strip()
                self.api_cancel_task(task_id)
            elif choice == '15':
                self.print_dispatches()
                task_id = input('Task ID to cancel: ').strip()
                self.cancel_task(task_id, requester=requester)
            elif choice == '16':
                self.pause_all_robots()
            elif choice == '17':
                self.resume_all_robots()
            elif choice == '18':
                self.reset_all_robots()
            elif choice == '19':
                self.run_showcase(
                    requester=requester,
                    robot_type=robot_type,
                    num_loops=num_loops,
                )
            else:
                print('Unknown choice. Pick 1-19, or q.')

        return False

    def submit_menu_route(
        self,
        label,
        start_name,
        finish_name,
        requester='tutorial',
        robot_type='',
        num_loops=1,
    ):
        task_id = f'{label}_{int(time.time())}'
        return self.submit_loop_task(
            task_id,
            start_name,
            finish_name,
            requester=requester,
            robot_type=robot_type,
            num_loops=num_loops,
        )


def make_parser():
    parser = argparse.ArgumentParser(
        description='Submit simple learning tasks to Open-RMF.',
    )
    parser.add_argument(
        'command',
        choices=[
            'route_a',
            'route_b',
            'route_c',
            'route_d',
            'route_e',
            'route_f',
            'route_g',
            'route_h',
            'route_i',
            'route_j',
            'route_k',
            'route_l',
            'both',
            'conflict',
            'all',
            'loop',
            'cancel',
            'api_cancel',
            'dispatches',
            'pause_all',
            'resume_all',
            'reset_all',
            'showcase',
            'status',
            'menu',
        ],
        help=(
            'route_a sends pickup -> dropoff, route_b sends '
            'pickup_B -> dropoff_B, route_c sends inspection -> storage, '
            'route_d sends storage -> dropoff_B, and route_e through '
            'route_l cover receiving, shipping, racks, staging, returns, '
            'maintenance, and battery swap. both/conflict sends the two '
            'shared-zone routes, and all sends every route so RMF can choose '
            'across the 4 AGVs. loop uses custom waypoint names. cancel '
            'cancels a dispatch task ID using the dispatcher service. '
            'api_cancel cancels a task ID using the task API. dispatches '
            'lists dispatcher task IDs. pause_all/resume_all control the '
            'fake robot API hold state. reset_all returns robots to their '
            'charger poses. showcase runs the video demo sequence. status '
            'prints /fleet_states. menu opens an interactive terminal menu.'
        ),
    )
    parser.add_argument(
        'start_name',
        nargs='?',
        help='Start waypoint for loop, or task ID for cancel/api_cancel.',
    )
    parser.add_argument(
        'finish_name',
        nargs='?',
        help='Finish waypoint for the custom loop command.',
    )
    parser.add_argument(
        '--task-id',
        default=None,
        help='Optional task label. Defaults to a generated learning ID.',
    )
    parser.add_argument(
        '--requester',
        default='tutorial',
        help='Name of the requester sent to RMF.',
    )
    parser.add_argument(
        '--robot-type',
        default='',
        help='Optional robot type/fleet filter. Empty lets RMF choose.',
    )
    parser.add_argument(
        '--num-loops',
        type=int,
        default=1,
        help='How many times the loop should run.',
    )
    return parser


def jobs_from_args(args, parser):
    if args.command in (
        'status',
        'menu',
        'cancel',
        'api_cancel',
        'dispatches',
        'pause_all',
        'resume_all',
        'reset_all',
        'showcase',
    ):
        return []

    if args.command in ('both', 'conflict'):
        routes = CONFLICT_ROUTES
    elif args.command == 'all':
        routes = ALL_ROUTES
    elif args.command == 'loop':
        if not args.start_name or not args.finish_name:
            parser.error('loop requires START_NAME and FINISH_NAME')
        routes = [('loop', args.start_name, args.finish_name)]
    else:
        routes = [(args.command, *ROUTES[args.command])]

    base_task_id = args.task_id or f'{args.command}_{int(time.time())}'
    jobs = []
    for label, start_name, finish_name in routes:
        task_id = base_task_id
        if len(routes) > 1:
            task_id = f'{base_task_id}_{label}'
        jobs.append((task_id, start_name, finish_name))
    return jobs


def main():
    parser = make_parser()
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    jobs = jobs_from_args(args, parser)

    rclpy.init(args=sys.argv)
    node = SimpleTaskClient()
    try:
        if args.command == 'status':
            ok = node.print_status()
            return 0 if ok else 1

        if args.command == 'dispatches':
            ok = node.print_dispatches()
            return 0 if ok else 1

        if args.command == 'pause_all':
            ok = node.pause_all_robots()
            return 0 if ok else 1

        if args.command == 'resume_all':
            ok = node.resume_all_robots()
            return 0 if ok else 1

        if args.command == 'reset_all':
            ok = node.reset_all_robots()
            return 0 if ok else 1

        if args.command == 'showcase':
            ok = node.run_showcase(
                requester=args.requester,
                robot_type=args.robot_type,
                num_loops=args.num_loops,
            )
            return 0 if ok else 1

        if args.command == 'menu':
            try:
                ok = node.run_menu(
                    requester=args.requester,
                    robot_type=args.robot_type,
                    num_loops=args.num_loops,
                )
            except (EOFError, KeyboardInterrupt):
                print()
                ok = True
            return 0 if ok else 1

        if args.command == 'cancel':
            if not args.start_name:
                parser.error('cancel requires TASK_ID')
            ok = node.cancel_task(args.start_name, requester=args.requester)
            return 0 if ok else 1

        if args.command == 'api_cancel':
            if not args.start_name:
                parser.error('api_cancel requires TASK_ID')
            ok = node.api_cancel_task(args.start_name)
            return 0 if ok else 1

        ok = True
        for task_id, start_name, finish_name in jobs:
            ok = node.submit_loop_task(
                task_id,
                start_name,
                finish_name,
                requester=args.requester,
                robot_type=args.robot_type,
                num_loops=args.num_loops,
            ) and ok
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
