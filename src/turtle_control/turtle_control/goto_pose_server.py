import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits

DIST_TOL = 0.1
YAW_TOL = 0.05
K_LIN = 1.5
K_ANG = 4.0
K_YAW = 3.0
LOOP_HZ = 50.0


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('goto_pose_server')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters (read live every loop)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.add_on_set_parameters_callback(self.on_param_change)

        # Pose / cmd_vel
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10,
                                 callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Speed limits service
        self.create_service(SetSpeedLimits, 'set_speed_limits',
                            self.handle_set_limits,
                            callback_group=self.cb_group)

        # Action server
        self.action_server = ActionServer(
            self, GoToPose, 'goToPose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb_group)

        self.get_logger().info('goToPose server ready')

    # ---------- parameters ----------
    def on_param_change(self, params):
        for p in params:
            if p.name in ('max_linear_speed', 'max_angular_speed'):
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='Speed must be > 0')
        return SetParametersResult(successful=True)

    def handle_set_limits(self, request, response):
        if request.max_linear_speed <= 0.0 or request.max_angular_speed <= 0.0:
            response.success = False
            response.message = 'Speeds must be greater than zero'
            return response
        result = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE,
                      float(request.max_linear_speed)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE,
                      float(request.max_angular_speed)),
        ])
        if all(r.successful for r in result):
            response.success = True
            response.message = 'Speed limits updated'
        else:
            response.success = False
            response.message = 'Failed to update parameters'
        return response

    # ---------- pose ----------
    def on_pose(self, msg):
        self.pose = msg

    # ---------- action callbacks ----------
    def on_goal(self, goal_request):
        t = goal_request.target
        if not (0.0 <= t.x <= 11.0 and 0.0 <= t.y <= 11.0):
            self.get_logger().warn('Goal rejected: outside window (0..11)')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def stop(self):
        self.cmd_pub.publish(Twist())

    def execute(self, goal_handle):
        target = goal_handle.request.target
        self.get_logger().info(
            f'New goal: x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.2f}')

        # wait for first pose
        while self.pose is None:
            time.sleep(0.05)

        feedback = GoToPose.Feedback()
        result = GoToPose.Result()
        period = 1.0 / LOOP_HZ

        # Phase 1: go to position
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success = False
                return result

            max_lin = self.get_parameter('max_linear_speed').value
            max_ang = self.get_parameter('max_angular_speed').value

            dx = target.x - self.pose.x
            dy = target.y - self.pose.y
            dist = math.hypot(dx, dy)
            if dist < DIST_TOL:
                break

            angle_to_goal = math.atan2(dy, dx)
            angle_err = normalize(angle_to_goal - self.pose.theta)

            cmd = Twist()
            cmd.angular.z = clamp(K_ANG * angle_err, max_ang)
            # slow down linear motion while still turning
            if abs(angle_err) < 0.5:
                cmd.linear.x = clamp(K_LIN * dist, max_lin)
            self.cmd_pub.publish(cmd)

            feedback.distance_remaining = float(dist)
            feedback.yaw_remaining = float(
                normalize(target.yaw - self.pose.theta))
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        self.stop()

        # Phase 2: rotate to final yaw
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success = False
                return result

            max_ang = self.get_parameter('max_angular_speed').value
            yaw_err = normalize(target.yaw - self.pose.theta)
            if abs(yaw_err) < YAW_TOL:
                break

            cmd = Twist()
            cmd.angular.z = clamp(K_YAW * yaw_err, max_ang)
            self.cmd_pub.publish(cmd)

            feedback.distance_remaining = float(
                math.hypot(target.x - self.pose.x, target.y - self.pose.y))
            feedback.yaw_remaining = float(yaw_err)
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        self.stop()

        result.success = True
        result.distance_error = float(
            math.hypot(target.x - self.pose.x, target.y - self.pose.y))
        result.yaw_error = float(abs(normalize(target.yaw - self.pose.theta)))
        goal_handle.succeed()
        self.get_logger().info('Goal reached')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
