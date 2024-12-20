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
        self._verbose = verbose
        self._work_space = ((-400, 400), (300, 1000), (200, 700))

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

        # listen to height command
        self._height_sub = self.create_subscription(
            rosmsg_geo.PoseStamped,
            "/rotated_end_effector_pose",
            self._on_receive_pose,
            1
        )
        self._robot_position = None
        self._probe_position = None
        self._position_offset = None # robot = cam + offset

        # # listen to height command
        # self._height_sub = self.create_subscription(
        #     rosmsg_std.String,
        #     "/probe_height",
        #     self._on_receive_height,
        #     1
        # )
        # self._robot_height = None
        # self._probe_height = None
        # self._height_offset = None # robot = cam + offset


    
    def _on_receive_pose(self, msg: rosmsg_geo.PoseStamped):        
        self._probe_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if self._position_offset is None:
            starting_height_cam = self._probe_position
            self._position_offset = self._robot_position - starting_height_cam
            if self._verbose:
                print(f"Setting referencing height {starting_height_cam}(Cam) = {self._robot_position}(Robot)")
                print(f"height offset = {self._position_offset}")
            # TODO: check if valid
    
    def _on_receive_height(self, msg: rosmsg_std.String):
        self._probe_height = float(msg.data)
        if self._height_offset is None:
            start = self._probe_height
            self._height_offset = self._robot_height - start


    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        if self._lbr_state is None:
            self._lbr_state = lbr_state

        robot_position = np.array(self._lbr_state.measured_cartesian_pose[:3]) # height
        # robot_height = self._lbr_state.measured_cartesian_pose[2]
        # if 100.0 < robot_height < 800.0:
        #     self._robot_height = robot_height
        # elif self._verbose:
        #     print(f"robot height = {robot_height} is not valid")

        # validate
        if all(self._work_space[i][0] < robot_position[i] < self._work_space[i][1] for i in range(3)):
            self._robot_position = robot_position
        elif self._verbose:
            print(f"robot height = {robot_position} is not valid")

        if self._dt is None:
            print("self._dt is not set")
            return        

        # print("ipo cart pose", self._lbr_state.ipo_cartesian_pose)
        # print("measured cart pose", self._lbr_state.measured_cartesian_pose)

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            self._lbr_cartesian_pose_command.cartesian_pose_quaternion = deepcopy(
                self._lbr_state.measured_cartesian_pose
            )
            if any([x is None for x in self._lbr_cartesian_pose_command.cartesian_pose_quaternion]):
                print(f"IPO pose not valid: {self._lbr_cartesian_pose_command.cartesian_pose_quaternion}")
                return
            
            # if self._height_offset is None or self._probe_height is None:
            #     return
            if self._position_offset is None or self._probe_position is None:
                return
            
            # height_cmd = self._height_offset + self._probe_height
            height_cmd = self._position_offset + self._probe_position
            
            # TODO
            # height_cmd =  max(self.height_limit[0], min(self.height_limit[1], height_cmd))
            deviation = height_cmd - self._lbr_cartesian_pose_command.cartesian_pose_quaternion[:3]
            if self._verbose:
                print("deviation   ",  deviation)
            
            # only height
            self._lbr_cartesian_pose_command.cartesian_pose_quaternion[:3] = height_cmd
            if self._verbose:
                print("publishing position",  
                    self._lbr_cartesian_pose_command.cartesian_pose_quaternion[2])

            if all(self._work_space[i][0] < height_cmd[i] < self._work_space[i][1] for i in range(3)):
                # pass
                # sandbox
                self._lbr_cartesian_pose_command_pub.publish(
                    self._lbr_cartesian_pose_command
                )
            elif self._verbose:
                print(f"height_cmd = {height_cmd} is not valid")
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
    rclpy.spin(CartesianSineOverlayNode("cart_sine_overlay_node", verbose=True))
    rclpy.shutdown()
