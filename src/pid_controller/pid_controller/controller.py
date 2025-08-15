import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class PID_controller(Node):
    def __init__(self):
        super().__init__("PID_controller")

        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("joint_name", "joint_name")

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value
        self.joint_name = self.get_parameter("joint_name").value

        self.get_logger().info(
            f"Initial parameters (kp, ki, kd): {self.kp}, {self.ki}, {self.kd}, joint_name: {self.joint_name}"
        )

        self.js_subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 1
        )
        self.target_sub = self.create_subscription(
            Float64, "/joint_setpoint", self.setpoint_callback, 1
        )
        self.cmd_publisher = self.create_publisher(
            Float64MultiArray, "/position_control/commands", 1
        )

        self.initialized = False
        self.current_position = None
        self.target_point = None
        self.cmd = None
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.ctrl_loop_timer = self.create_timer(0.01, self.ctrl_loop)

    def setpoint_callback(self, msg):
        self.target_point = msg.data

    def joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            index = msg.name.index(self.joint_name)
            self.current_position = msg.position[index]

            if not self.initialized:
                self.target_point = self.current_position
                self.cmd = self.current_position
                self.initialized = True

    def ctrl_loop(self):

        dt = 0.01
        error = 0.0
        integral_error = 0.0
        prev_error = 0.0

        msg = Float64MultiArray()

        if self.initialized and self.target_point is not None:

            error = self.target_point - self.cmd
            integral_error = integral_error + error * dt
            derivative_error = (error - prev_error) / dt
            c = self.kp * error + self.ki * integral_error + self.kd * derivative_error
            self.cmd += c * dt
            msg.data = [self.cmd]

            prev_error = error

            self.cmd_publisher.publish(msg)

            self.get_logger().info(
                f"Target: {self.target_point}, error: {error}, Command: {self.cmd}"
            )

    def parameter_callback(self, params):
        for param in params:
            if param.name == "kp":
                self.kp = param.value
            elif param.name == "ki":
                self.ki = param.value
            elif param.name == "kd":
                self.kd = param.value

        print("New parameters set: ", self.kp, self.ki, self.kd)

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    joint_position_controller = PID_controller()
    rclpy.spin(joint_position_controller)
    joint_position_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
