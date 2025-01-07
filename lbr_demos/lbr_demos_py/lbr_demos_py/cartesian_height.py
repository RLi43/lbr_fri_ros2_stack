import math
import numpy as np
from copy import deepcopy

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
import geometry_msgs.msg as rosmsg_geo
import std_msgs.msg as rosmsg_std

# import lbr_fri_idl
from lbr_fri_idl.msg import LBRCartesianPoseCommand, LBRState


class CartesianSineOverlayNode(Node):
    def __init__(self, node_name: str, verbose = False) -> None:
        super().__init__(node_name)
        self._position_deviation_eps = 0.001
        self._verbose = verbose
        self._work_space = ((-400, 400), (300, 1000), (200, 700))

        # create publisher to command/joint_position
        self._lbr_cartesian_pose_command_pub = self.create_publisher(
            LBRCartesianPoseCommand,
            "command/cartesian_pose",
            1,
        )

        # create subscription to robot state
        self._lbr_state = None
        self._lbr_state_sub = self.create_subscription(
            LBRState, "state", self._on_lbr_state, 1
        )
        # get control rate from controller_manager
        self._dt = None
        self._retrieve_update_rate()

        # listen to data from camera
        self._camera_sub = self.create_subscription(
            rosmsg_geo.PoseStamped,
            "/rotated_end_effector_pose",
            self._on_receive_pose,
            1
        )
        self._robot_position = None
        self._probe_position = None
        self._position_offset = None # robot = cam + offset


    def _on_receive_pose(self, msg: rosmsg_geo.PoseStamped):        
        self._probe_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if self._position_offset is None:
            self._position_offset = self._robot_position - self._probe_position
            if self._verbose:
                print(f"Setting referencing height {self._probe_position}(Cam) = {self._robot_position}(Robot)")
                print(f"height offset = {self._position_offset}")
            # TODO: check if valid

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._robot_position = lbr_state.measured_cartesian_pose[:3]

        if self._dt is None:
            print("self._dt is not set")
            return        

        # print("ipo cart pose", self._lbr_state.ipo_cartesian_pose)
        # print("measured cart pose", self._lbr_state.measured_cartesian_pose)

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4

            if self._position_offset is None or self._probe_position is None:
                return
            target_position = self._position_offset + self._probe_position
            if np.any(np.abs(target_position - self._robot_position) > self._position_deviation_eps):
                # TODO: Target validate
                if all(self._work_space[i][0] < target_position[i] < self._work_space[i][1] for i in range(3)):
                    pass
                else:
                    print(f"target_position = {target_position} is not valid")
                    return

                cart_pose_cmd = LBRCartesianPoseCommand()
                cart_pose_cmd.cartesian_pose_quaternion = deepcopy(
                    lbr_state.measured_cartesian_pose
                )
                if any([x is None for x in cart_pose_cmd.cartesian_pose_quaternion]):
                    print(f"MEASURED pose not valid: {cart_pose_cmd.cartesian_pose_quaternion}")
                    return
            
                cart_pose_cmd.cartesian_pose_quaternion[:3] = target_position
                if self._verbose:
                    print("publishing position", cart_pose_cmd.cartesian_pose_quaternion[:3])

                # sandbox
                self._lbr_cartesian_pose_command_pub.publish(
                    cart_pose_cmd
                )
        else:
            # reset 
            
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
    rclpy.spin(CartesianSineOverlayNode("cart_follow_node", verbose=True))
    rclpy.shutdown()
