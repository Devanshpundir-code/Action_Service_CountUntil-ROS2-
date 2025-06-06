import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil


class CountActionClient(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.count_until_client = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_value_by_user, delay):
        self.get_logger().info("Waiting for server...")
        self.count_until_client.wait_for_server()

        goal_msg = CountUntil.Goal()
        goal_msg.target_number = target_value_by_user
        goal_msg.delay = delay

        self.get_logger().info("Sending goal request...")


        self.future = self.count_until_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: Current number = {feedback.current_number}")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: Reached number = {result.reached_number}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CountActionClient("count_until_client_node")
    node.send_goal(5, 1)  
    rclpy.spin(node)


if __name__ == '__main__':
    main()
