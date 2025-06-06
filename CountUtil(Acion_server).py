import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil  


class MyActionServer(Node):
    def __init__(self):
        super().__init__("action_server_node")
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback
        )

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal. Checking if we should accept or reject it.")
        if goal_request.target_number <= 0:
            self.get_logger().info("Target number is less than or equal to 0. Rejecting.")
            return GoalResponse.REJECT
        self.get_logger().info("Goal accepted.")
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing the goal...")

        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        counter = 0

        feedback_msg = CountUntil.Feedback()

        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled.")
                return CountUntil.Result()

            counter += 1
            self.get_logger().info(f"Count: {counter}")

            feedback_msg.current_number = counter
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(delay)

        goal_handle.succeed()
        result = CountUntil.Result()
        result.reached_number = counter
        self.get_logger().info("Goal succeeded.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MyActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
