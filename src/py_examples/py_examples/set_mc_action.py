#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcAction, SetMcInputSource
from aimdk_msgs.msg import (
    RequestHeader,
    CommonState,
    McAction,
    McActionCommand,
    McInputAction,
)


class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.action_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction'
        )
        self.input_source_client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource'
        )
        self.source_name = 'node'
        self.source_priority = 40
        self.source_timeout_ms = 1000
        self.get_logger().info('✅ SetMcAction client node created.')

        # Wait for the service to become available
        while not self.action_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ SetMcAction service unavailable, waiting...')
        while not self.input_source_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ SetMcInputSource service unavailable, waiting...')

        self.get_logger().info('🟢 Services available, ready to send request.')

    def _call_with_retry(self, client, req, timeout_sec: float = 0.25):
        future = None
        for i in range(8):
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                return future.result()
            self.get_logger().info(f'trying ... [{i}]')
        return None

    def _register_input_source(self) -> bool:
        req = SetMcInputSource.Request()
        req.request.header = RequestHeader()
        req.action = McInputAction()
        req.action.value = McInputAction.INPUTACTION_ADD
        req.input_source.name = self.source_name
        req.input_source.priority = self.source_priority
        req.input_source.timeout = self.source_timeout_ms

        req.request.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'📨 Registering input source: name={self.source_name}, '
            f'priority={self.source_priority}, timeout={self.source_timeout_ms}ms'
        )
        response = self._call_with_retry(self.input_source_client, req)
        if response is None:
            self.get_logger().error('❌ Input source registration timed out.')
            return False

        if response.response.header.code == 0:
            return True

        req.action.value = McInputAction.INPUTACTION_MODIFY
        req.request.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'Input source ADD failed with code={response.response.header.code}, trying MODIFY'
        )
        response = self._call_with_retry(self.input_source_client, req)
        if response is None:
            self.get_logger().error('❌ Input source modify timed out.')
            return False
        if response.response.header.code != 0:
            self.get_logger().error(
                f'❌ Failed to register input source. code={response.response.header.code}'
            )
            return False
        return True

    def send_request(self, action_name: str):
        if not self._register_input_source():
            return

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = self.source_name

        cmd = McActionCommand()
        cmd.action_desc = action_name
        req.command = cmd

        self.get_logger().info(
            f'📨 Sending request to set robot mode: {action_name} '
            f'(source={self.source_name})'
        )
        req.header.stamp = self.get_clock().now().to_msg()
        response = self._call_with_retry(self.action_client, req)
        if response is None:
            self.get_logger().error('❌ Service call failed or timed out.')
            return

        if response.response.status.value == CommonState.SUCCESS:
            self.get_logger().info('✅ Robot mode set successfully.')
        else:
            self.get_logger().error(
                f'❌ Failed to set robot mode: {response.response.message}'
            )


def main(args=None):
    action_info = {
        'PASSIVE_DEFAULT': ('PD', 'joints with zero torque'),
        'DAMPING_DEFAULT': ('DD', 'joints in damping mode'),
        'JOINT_DEFAULT': ('JD', 'Position Control Stand (joints locked)'),
        'STAND_DEFAULT': ('SD', 'Stable Stand (auto-balance)'),
        'LOCOMOTION_DEFAULT': ('LD', 'locomotion mode (walk or run)'),
    }

    choices = {}
    for k, v in action_info.items():
        choices[v[0]] = k

    rclpy.init(args=args)
    node = None
    try:
        # Prefer command-line argument, otherwise prompt for input
        if len(sys.argv) > 1:
            motion = sys.argv[1]
        else:
            print('{:<4} - {:<20} : {}'.format('abbr',
                  'robot mode', 'description'))
            for k, v in action_info.items():
                print(f'{v[0]:<4} - {k:<20} : {v[1]}')
            motion = input('Enter abbr of robot mode:')

        action_name = choices.get(motion)
        if not action_name:
            raise ValueError(f'Invalid abbr of robot mode: {motion}')

        node = SetMcActionClient()
        node.send_request(action_name)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
