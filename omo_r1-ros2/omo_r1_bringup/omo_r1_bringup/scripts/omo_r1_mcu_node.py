#!/usr/bin/env python3

"""ROS 2 Jazzy MCU node for the OMO R1 platform."""

from dataclasses import dataclass, field
import math
from time import sleep
from typing import Any, List, Optional

import rclpy
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from omo_r1_interfaces.srv import Battery, Calg, Color, Onoff, ResetOdom

from .calc.quaternion_from_euler import quaternion_from_euler
from .omo_packet_handler import PacketHandler


DEFAULT_PARAMETERS = [
    ('port.name', '/dev/ttyMotor'),
    ('port.baudrate', 115200),
    ('motor.gear_ratio', 15.0),
    ('motor.max_lin_vel_x', 0.6),
    ('motor.max_ang_vel_z', 0.5),
    ('wheel.separation', 0.57),
    ('wheel.radius', 0.1),
    ('sensor.enc_pulse', 60000.0),
    ('sensor.use_gyro', False),
]


@dataclass
class OdomPose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    timestamp: Any = None
    pre_timestamp: Any = None


@dataclass
class OdomVel:
    x: float = 0.0
    y: float = 0.0
    w: float = 0.0


@dataclass
class Joint:
    joint_name: List[str] = field(default_factory=lambda: ['wheel_left_joint', 'wheel_right_joint'])
    joint_pos: List[float] = field(default_factory=lambda: [0.0, 0.0])
    joint_vel: List[float] = field(default_factory=lambda: [0.0, 0.0])


class ComplementaryFilter:
    """Fuse gyro and wheel odometry for yaw estimation."""

    def __init__(self, node_logger) -> None:
        self._logger = node_logger
        self.theta = 0.0
        self.pre_theta = 0.0
        self.wheel_ang = 0.0
        self.filter_coef = 2.5
        self.gyro_bias = 0.0
        self.count_for_gyro_bias = 110

    def gyro_calibration(self, gyro: float) -> str:
        self.count_for_gyro_bias -= 1

        if self.count_for_gyro_bias > 100:
            return "Prepare for gyro_calibration"

        self.gyro_bias += gyro
        if self.count_for_gyro_bias == 1:
            self.gyro_bias /= 100
            self._logger.info('Complete : Gyro calibration')
            return "gyro_calibration OK"

        return "During gyro_calibration"

    def calc_filter(self, gyro: float, d_time: float) -> float:
        if self.count_for_gyro_bias != 1:
            self.gyro_calibration(gyro)
            return 0.0

        gyro -= self.gyro_bias
        self.pre_theta = self.theta
        temp = -1 / self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
        self.theta = self.pre_theta + temp * d_time
        return self.theta


class OMOR1MiniNode(Node):
    """Primary MCU control node for the OMO R1 base."""

    def __init__(self) -> None:
        super().__init__('omo_r1_motor_setting')
        self._logger = self.get_logger()

        self._declare_default_parameters()
        port_name = self.get_parameter('port.name').value
        port_baudrate = self.get_parameter('port.baudrate').value

        self.gear_ratio = float(self.get_parameter('motor.gear_ratio').value)
        self.wheel_separation = float(self.get_parameter('wheel.separation').value)
        self.wheel_radius = float(self.get_parameter('wheel.radius').value)
        self.enc_pulse = float(self.get_parameter('sensor.enc_pulse').value)
        self.use_gyro = bool(self.get_parameter('sensor.use_gyro').value)

        self.distance_per_pulse = 2 * math.pi * self.wheel_radius / self.enc_pulse / self.gear_ratio
        self._logger.info(
            f'Loaded parameters: gear_ratio={self.gear_ratio:.3f}, '
            f'wheel_separation={self.wheel_separation:.3f}, '
            f'wheel_radius={self.wheel_radius:.3f}, '
            f'enc_pulse={self.enc_pulse:.1f}, '
            f'distance_per_pulse={self.distance_per_pulse:.6f}'
        )

        self.ph = PacketHandler(port_name, port_baudrate, self._logger)
        self.calc_yaw = ComplementaryFilter(self._logger)

        self.ph.robot_state = {
            "VW": [0.0, 0.0],
            "ODO": [0.0, 0.0],
            "ACCL": [0.0, 0.0, 0.0],
            "GYRO": [0.0, 0.0, 0.0],
            "POSE": [0.0, 0.0, 0.0],
            "BAT": [0.0, 0.0, 0.0],
        }
        self.ph.incomming_info = ['ODO', 'VW', "POSE", "GYRO"]
        self.ph.set_periodic_info(50)

        self.max_lin_vel_x = float(self.get_parameter('motor.max_lin_vel_x').value)
        self.max_ang_vel_z = float(self.get_parameter('motor.max_ang_vel_z').value)

        self.odom_pose = OdomPose()
        now = self.get_clock().now()
        self.odom_pose.timestamp = now
        self.odom_pose.pre_timestamp = now
        self.odom_vel = OdomVel()
        self.joint = Joint()
        self.d_setHDLT = {'switch': 0}

        self.create_service(Onoff, 'set_headlight', self.cbSrv_headlight)
        self.create_service(Color, 'set_rgbled', self.cbSrv_setColor)
        self.create_service(ResetOdom, 'reset_odom', self.cbSrv_resetODOM)
        self.create_service(Battery, 'check_battery', self.cbSrv_checkBattery)

        self.create_subscription(Twist, 'cmd_vel', self.cbCmdVelMsg, 10)

        self.pub_JointStates = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_IMU = self.create_publisher(Imu, 'imu', 10)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, 'pose', 10)

        sleep(0.01)
        self.ph.set_periodic_info(50)

        self.timerProc = self.create_timer(0.01, self.update_robot)
        self._prev_odo_l: Optional[float] = None
        self._prev_odo_r: Optional[float] = None
        self._prev_update_time: Optional[rclpy.time.Time] = None
        self._odo_diff_threshold_mm = 10.0  # Ignore tiny odometry jitter (noise guard)

    def _declare_default_parameters(self) -> None:
        for param_name, default_value in DEFAULT_PARAMETERS:
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_value)

    def convert2odo_from_each_wheel(self, enc_l, enc_r):
        return enc_l * self.distance_per_pulse, enc_r * self.distance_per_pulse

    def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel, vel_z):
        odo_l /= 1000.0
        odo_r /= 1000.0

        self.odom_pose.timestamp = self.get_clock().now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp

        trans_vel /= 1000.0
        orient_vel /= 1000.0

        if self.use_gyro:
            self.calc_yaw.wheel_ang += orient_vel * dt
            self.odom_pose.theta = self.calc_yaw.calc_filter(vel_z * math.pi / 180.0, dt)
            self._logger.debug(
                (
                    f'omo_r1 state : whl pos {odo_l:.2f}, {odo_r:.2f}, '
                    f'gyro : {vel_z:.2f}, '
                    f'whl odom : {self.calc_yaw.wheel_ang * 180 / math.pi:.2f}, '
                    f'robot theta : {self.odom_pose.theta * 180 / math.pi:.2f}'
                )
            )
        else:
            self.odom_pose.theta += orient_vel * dt

        d_x = trans_vel * math.cos(self.odom_pose.theta)
        d_y = trans_vel * math.sin(self.odom_pose.theta)
        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt
        self._logger.debug(
            (
                f'ODO L:{odo_l:.2f}, R:{odo_r:.2f}, V:{trans_vel:.2f}, W={orient_vel:.2f} -> '
                f'X:{self.odom_pose.x:.2f}, Y:{self.odom_pose.y:.2f}, Theta:{self.odom_pose.theta:.2f}'
            )
        )
        q = quaternion_from_euler(0, 0, self.odom_pose.theta)

        self.odom_vel.x = trans_vel
        self.odom_vel.y = 0.0
        self.odom_vel.w = orient_vel

        timestamp_now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = timestamp_now
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = trans_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = orient_vel
        self.pub_Odom.publish(odom)

        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

    def updatePoseStates(self, roll, pitch, yaw):
        pose = Pose()
        pose.orientation.x = roll
        pose.orientation.y = pitch
        pose.orientation.z = yaw
        self.pub_pose.publish(pose)

    def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
        odo_l /= 1000.0
        odo_r /= 1000.0

        wheel_ang_left = odo_l / self.wheel_radius
        wheel_ang_right = odo_r / self.wheel_radius

        wheel_ang_vel_left = (trans_vel - (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius
        wheel_ang_vel_right = (trans_vel + (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius

        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

        timestamp_now = self.get_clock().now().to_msg()

        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = timestamp_now
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []
        self.pub_JointStates.publish(joint_states)

    def update_robot(self):
        self.ph.read_packet()
        odo_l = self.ph._wodom[0]
        odo_r = self.ph._wodom[1]
        trans_vel = self.ph._vel[0]
        orient_vel = self.ph._vel[1]
        vel_z = self.ph._gyro[2]
        roll_imu = self.ph._imu[0]
        pitch_imu = self.ph._imu[1]
        yaw_imu = self.ph._imu[2]

        now = self.get_clock().now()
        dt = None
        if self._prev_update_time is not None:
            dt = (now - self._prev_update_time).nanoseconds * 1e-9
        self._prev_update_time = now

        if self._prev_odo_l is None or self._prev_odo_r is None:
            self._prev_odo_l = odo_l
            self._prev_odo_r = odo_r
        elif dt and dt > 0.0:
            odo_l_diff = odo_l - self._prev_odo_l
            odo_r_diff = odo_r - self._prev_odo_r
            movement_detected = (
                abs(odo_l_diff) >= self._odo_diff_threshold_mm
                or abs(odo_r_diff) >= self._odo_diff_threshold_mm
            )
            if abs(trans_vel) < 1e-3 and movement_detected:
                # Reconstruct linear velocity (mm/s) from wheel odometry when VW data is missing.
                lin_mm_per_s = (odo_l_diff + odo_r_diff) / (2.0 * dt)
                trans_vel = lin_mm_per_s
            if abs(orient_vel) < 1e-3 and movement_detected:
                # Likewise recover angular velocity (converted to mrad/s) from left/right differences.
                ang_rad_per_s = (
                    (odo_r_diff - odo_l_diff) / (self.wheel_separation * 1000.0)
                ) / dt
                orient_vel = ang_rad_per_s * 1000.0
            if not movement_detected:
                trans_vel = 0.0
                orient_vel = 0.0
            self._prev_odo_l = odo_l
            self._prev_odo_r = odo_r
        else:
            self._prev_odo_l = odo_l
            self._prev_odo_r = odo_r
        self.ph._vel = [trans_vel, orient_vel]

        self.update_odometry(odo_l, odo_r, trans_vel, orient_vel, vel_z)
        self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)
        self.updatePoseStates(roll_imu, pitch_imu, yaw_imu)

    def cbCmdVelMsg(self, cmd_vel_msg):
        lin_vel_x = cmd_vel_msg.linear.x
        ang_vel_z = cmd_vel_msg.angular.z

        lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, lin_vel_x))
        ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, ang_vel_z))
        self.ph.write_base_velocity(lin_vel_x * 1000.0, ang_vel_z * 1000.0)

    def cbSrv_headlight(self, request, response):
        onoff = '1' if request.set else '0'
        self.d_setHDLT['switch'] = int(request.set)
        command = "$cHDLT," + onoff
        self.ph.write_port(command)
        self._logger.info(f'SERVICE: Headlight: {onoff}')
        return response

    def cbSrv_setColor(self, request, response):
        self._logger.info(
            f"SERVICE: SET COLOR: R({request.red})G({request.green})B({request.blue})"
        )
        return response

    def cbSrv_checkBattery(self, request, response):
        self.ph.update_battery_state()
        bat_status = self.ph.robot_state['BAT']
        if len(bat_status) == 3:
            self._logger.info(
                f"SERVICE: Battery V:{bat_status[0]}, SOC: {bat_status[1]}, Current {bat_status[2]}"
            )
            response.volt = bat_status[0] * 0.1
            response.soc = bat_status[1]
            response.current = bat_status[2] * 0.001
        return response

    def cbSrv_resetODOM(self, request, response):
        self.odom_pose.x = request.x
        self.odom_pose.y = request.y
        self.odom_pose.theta = request.theta
        self._logger.info(
            f"SERVICE: RESET ODOM X:{request.x}, Y:{request.y}, Theta:{request.theta}"
        )
        return response

    def cbSrv_setBuzzer(self, request, response):
        onoff = '1' if request.set else '0'
        command = "$sBUZEN," + onoff
        self._logger.info(f"SERVICE: SET BUZZER : {onoff}")
        self.ph.write_port(command)
        return Onoff.Response()

    def calibrate_gyro(self, request):
        command = "$sCALG,1"
        self._logger.info("SERVICE: CALIBRATE GYRO")
        self.ph.write_port(command)
        return Calg.Response()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OMOR1MiniNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down omo_r1_mcu_node (Ctrl-C)')
    finally:
        node.ph.close_port()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
