#!/usr/bin/env python3
"""Command full pad-to-pad opening in mm; Gazebo Classic simulation only."""
import argparse
import math


def target_position(opening_mm, duration):
    if not math.isfinite(opening_mm) or not 0.0 <= opening_mm <= 50.0:
        raise ValueError('La apertura debe estar entre 0 y 50 mm.')
    if not math.isfinite(duration) or duration < 2.0:
        raise ValueError('La duración debe ser de al menos 2 segundos.')
    return opening_mm / 2000.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mm', type=float, required=True)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    try:
        position = target_position(args.mm, args.duration)
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = rclpy.create_node('hartech_set_opening', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    client = ActionClient(node, FollowJointTrajectory,
                          '/gripper_controller/follow_joint_trajectory')
    handle = None

    def wait(future, timeout):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('Sin respuesta del controlador; comprobar Gazebo y /clock.')
        result = future.result()
        if result is None:
            raise RuntimeError('El controlador no devolvió respuesta.')
        return result

    try:
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('gripper_controller no está disponible. Inicia el launch nuevo de Gazebo.')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['hartech_left_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        ns = round(args.duration * 1e9)
        point.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=3)
        handle = wait(client.send_goal_async(goal), 15.0)
        if not handle.accepted:
            raise RuntimeError('El controlador rechazó la orden.')
        result = wait(handle.get_result_async(), args.duration + 15.0)
        if (result.status != GoalStatus.STATUS_SUCCEEDED or
                result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(f'Falló la trayectoria: {result.result.error_string}')

        # Mensajes según la apertura solicitada
        if args.mm == 0:
            print('Gripper: cierre completado (0 mm).')
        elif args.mm == 50:
            print('Gripper: apertura completa (50 mm).')
        else:
            print(f'Gripper: apertura ajustada a {args.mm:g} mm.')

        handle = None
    except (Exception, KeyboardInterrupt):
        if handle is not None and handle.accepted and rclpy.ok():
            try:
                wait(handle.cancel_goal_async(), 3.0)
            except Exception:
                print('No se pudo confirmar la cancelación.')
        raise
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()