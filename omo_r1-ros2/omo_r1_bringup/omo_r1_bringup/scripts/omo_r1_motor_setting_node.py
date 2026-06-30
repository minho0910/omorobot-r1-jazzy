#!/usr/bin/env python3

from time import sleep

import rclpy
from rclpy.node import Node

from .driver.packet import Packet
from .driver.port import Port


DEFAULT_PARAMETERS = [
    ('port.name', '/dev/ttyMotor'),
    ('port.baudrate', 115200),
    ('motor.gear_ratio', 213),
    ('wheel.separation', 0.17),
    ('wheel.radius', 0.0335),
    ('sensor.enc_pulse', 44),
]


class OMOR1MiniMotorSettingNode(Node):
    def __init__(self) -> None:
        super().__init__('omo_r1_motor_setting')
        self._logger = self.get_logger()

        for param_name, default_value in DEFAULT_PARAMETERS:
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_value)

        port_name = self.get_parameter('port.name').value
        port_baudrate = int(self.get_parameter('port.baudrate').value)

        self.motor_gear_ratio = int(self.get_parameter('motor.gear_ratio').value)
        self.wheel_separation = float(self.get_parameter('wheel.separation').value)
        self.wheel_radius = float(self.get_parameter('wheel.radius').value)
        self.sensor_enc_pulse = int(self.get_parameter('sensor.enc_pulse').value)

        self.packet = Packet(Port(port_name, port_baudrate))

        self.set_gear_ratio()
        current_ratio = self.get_gear_ratio()
        self._logger.info(f'Configured gear ratio: {current_ratio}')

        self.run_motors()

    def set_gear_ratio(self) -> None:
        self.packet.write_data('GEAR', {'ratio': self.motor_gear_ratio})

    def get_gear_ratio(self) -> int:
        return self.packet.read_data('GEAR')['ratio']

    def run_motors(self, iterations: int = 6000) -> None:
        for _ in range(iterations):
            sleep(0.01)
            self.packet.write_data('RPM', {'left': 12, 'right': 12})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OMOR1MiniMotorSettingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
