import math
from copy import deepcopy

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

# import lbr_fri_idl
from lbr_fri_idl.msg import LBRCartesianPoseCommand, LBRState


class CartesianSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name, namespace='/lbr')

        self._eef_pose = [0.4, 0.0, 0.7, 0.5, 0.5, 0.5, 0.5]

        self._amplitude = 50  # mm
        self._frequency = 0.25  # Hz
        self._phase = 0.0
        self._axis = 2 # z
        self._lbr_cartesian_pose_command = LBRCartesianPoseCommand()

        # create publisher to command/joint_position
        self._lbr_cartesian_pose_command_pub = self.create_publisher(
            LBRCartesianPoseCommand,
            "command/cartesian_pose",
            1,
        )

        # create subscription to state
        self._lbr_state = None
        self._lbr_state_sub_ = self.create_subscription(
            LBRState, "state", self._on_lbr_state, 1
        )

        # get control rate from controller_manager
        self._dt = None
        self._retrieve_update_rate()

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        if self._dt is None:
            print("self._dt is not set")
            return
        if self._lbr_state is None:
            self._lbr_state = lbr_state
            self._initial_pose = self._lbr_state.measured_cartesian_pose

        self._lbr_cartesian_pose_command.cartesian_pose_quaternion = deepcopy(
            self._initial_pose
        )
        self._lbr_cartesian_pose_command.redundancy_value = 4.0 # not set

        print("measured cart pose", lbr_state.measured_cartesian_pose)
        print("measured redundancy value", lbr_state.measured_redundancy_value)

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay sine wave on 4th joint
            self._lbr_cartesian_pose_command.cartesian_pose_quaternion[
                self._axis
            ] += self._amplitude * math.sin(self._phase)
            print("publishing z axis with",  
                  self._lbr_cartesian_pose_command.cartesian_pose_quaternion[self._axis])
            self._phase += 2 * math.pi * self._frequency * self._dt

            self._lbr_cartesian_pose_command_pub.publish(
                self._lbr_cartesian_pose_command
            )
        else:
            # reset phase
            self._phase = 0.0
            print("Waiting for commanding_active")

    def _retrieve_update_rate(self) -> float:
        paramter_client = self.create_client(
            GetParameters, "controller_manager/get_parameters"
        )
        paramter_name = "update_rate"
        while not paramter_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self.get_logger().info(f"Waiting for {paramter_client.srv_name}...")
        future = paramter_client.call_async(
            GetParameters.Request(names=[paramter_name])
        )
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"Failed to get parameter '{paramter_name}'.")
        update_rate = future.result().values[0].integer_value
        self.get_logger().info(f"{paramter_name}: {update_rate} Hz")
        self._dt = 1.0 / float(update_rate)


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(CartesianSineOverlayNode("cart_sine_overlay_node"))
    rclpy.shutdown()
