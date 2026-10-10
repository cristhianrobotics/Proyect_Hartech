#!/usr/bin/env python3
"""Send a checked joint-space target to the Gazebo arm trajectory controller."""
import argparse
import math
from pathlib import Path
import sys
import time

import yaml


JOINTS = [f'joint_{i}' for i in range(1, 7)]


def select_target(config, pose, positions=None):
    if positions is not None:
        values = [float(value) for value in positions.split(',')]
    elif pose == 'zero':
        values = [0.0] * 6
    else:
        values = [float(config['joints'][joint]['home']) for joint in JOINTS]
        if pose == 'test':
            values[0] += math.radians(20.0)
            values[3] += math.radians(15.0)
            values[5] += math.radians(20.0)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('Provide exactly six finite positions in radians.')
    for joint, value in zip(JOINTS, values):
        limits = config['joints'][joint]
        if not float(limits['lower']) <= value <= float(limits['upper']):
            raise ValueError(f'{joint}: {value} is outside its angular limits.')
    return values


def validate_duration(current, target, simulation, duration):
    if not math.isfinite(duration) or duration <= 0.1:
        raise ValueError('Duration must be finite and greater than 0.1 seconds.')
    # Cubic interpolation with zero endpoint velocities has peak speed 1.5*dq/T.
    minimum = max(1.5 * abs(end - start) / simulation['joints'][joint]['velocity']
                  for joint, start, end in zip(JOINTS, current, target)) + 0.1
    if duration < minimum:
        raise ValueError(f'Duration is too short. Use at least {minimum:.3f} seconds.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', choices=['home', 'zero', 'test'], default='home')
    parser.add_argument('--positions', help='Six comma-separated angles in radians.')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')

    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from builtin_interfaces.msg import Duration
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    package = Path(get_package_share_directory('robot_description'))
    config = yaml.safe_load((package / 'config/robot_parameters.yaml').read_text())
    simulation = yaml.safe_load((package / 'config/gazebo_simulation.yaml').read_text())
    target = select_target(config, args.pose, args.positions)
    rclpy.init(args=ros_args)
    node = Node('send_robot_trajectory', parameter_overrides=[
        Parameter('use_sim_time', value=True),
    ])
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    observed = {}

    def receive_state(message):
        for joint, position in zip(message.name, message.position):
            observed[joint] = position

    subscription = node.create_subscription(JointState, '/joint_states', receive_state, 10)

    def duration_message(seconds):
        nanoseconds = round(seconds * 1_000_000_000)
        return Duration(sec=nanoseconds // 1_000_000_000,
                        nanosec=nanoseconds % 1_000_000_000)

    def await_future(future, timeout):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS operation timed out. Check that Gazebo is running and unpaused.')
        result = future.result()
        if result is None:
            raise RuntimeError('ROS operation returned no result.')
        return result

    try:
        if not client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError('arm_controller action server is unavailable. Start gazebo.launch.py first.')
        deadline = time.monotonic() + args.timeout
        while not all(joint in observed for joint in JOINTS):
            if time.monotonic() >= deadline:
                raise TimeoutError('No complete /joint_states message. Check the broadcaster and Gazebo pause state.')
            rclpy.spin_once(node, timeout_sec=0.1)
        current = [observed[joint] for joint in JOINTS]
        if not all(math.isfinite(value) for value in current):
            raise ValueError('Joint state contains non-finite positions.')
        validate_duration(current, target, simulation, args.duration)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        first = JointTrajectoryPoint()
        first.positions = current
        first.velocities = [0.0] * 6
        first.time_from_start = duration_message(0.1)
        last = JointTrajectoryPoint()
        last.positions = target
        last.velocities = [0.0] * 6
        last.time_from_start = duration_message(args.duration)
        goal.trajectory.points = [first, last]
        goal.goal_time_tolerance = duration_message(3.0)
        handle = await_future(client.send_goal_async(goal), args.timeout)
        if not handle.accepted:
            raise RuntimeError('Trajectory rejected by arm_controller.')
        node.get_logger().info(f'Moving to {target} in {args.duration} seconds.')
        result_future = handle.get_result_async()
        try:
            response = await_future(result_future, args.duration + args.timeout)
        except TimeoutError:
            try:
                await_future(handle.cancel_goal_async(), 3.0)
            except (TimeoutError, RuntimeError):
                node.get_logger().error('Could not confirm cancellation of the timed-out goal.')
            raise
        result = response.result
        if (response.status != GoalStatus.STATUS_SUCCEEDED
                or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(f'Trajectory failed: {result.error_code}: {result.error_string}')
        node.get_logger().info('Trajectory completed.')
    finally:
        node.destroy_subscription(subscription)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, TimeoutError) as error:
        print(f'Error: {error}', file=sys.stderr)
        sys.exit(1)
