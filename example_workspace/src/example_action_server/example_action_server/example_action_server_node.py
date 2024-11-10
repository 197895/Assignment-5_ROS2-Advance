
import time
from example_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import rclpy.action
from rclpy.node import Node


class ExampleActionServer(Node):

    def __init__(self):
        super().__init__('example_action_server')

        self._action_server = ActionServer(## 初始化一个ActionServer
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=## 开始执行动作时调用的函数
            goal_callback=## 接收到goal请求的时候调用的函数
            cancel_callback=## 接收到cancel请求的时候调用的函数)

    def goal_callback(self, goal_request):## 当客户端请求时，会调用这个函数
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')## 可以在这里决定是否接受客户端的请求
        return GoalResponse##是否接受客户端的请求

    def cancel_callback(self, goal_handle):## 当客户端请求取消时，会调用这个函数
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')## 可以在这里决定是否取消客户端的请求
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle:rclpy.action.server.ServerGoalHandle):##执行目标
        """Execute a goal."""
        self.get_logger().info('Executing goal...')## 执行目标

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()## 初始化Feedback对象
        feedback_msg.sequence = [0, 1]## 初始化Fibonacci序列

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:## 如果客户端请求取消，则取消目标
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback_msg.sequence.append(###斐波那契数列是怎么实现的？)

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

            # Publish the feedback
            goal_handle.publish_feedback(## 发布反馈feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = Fibonacci.Result()##初始化Result对象
        result.sequence = ## 直接把feedback的序列赋值给result的序列

        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result


def main(args=None):
    rclpy.init(args=args)
    example_action_server = ExampleActionServer()
    rclpy.spin(example_action_server)

if __name__ == '__main__':
    main()
