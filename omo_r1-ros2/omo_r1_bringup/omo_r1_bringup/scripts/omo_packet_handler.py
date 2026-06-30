#!/usr/bin/env python3

"""Serial packet handler for the OMO R1 MCU."""

import logging
from time import sleep
from typing import List

import serial

logger = logging.getLogger(__name__)


class PacketHandler:
    """Thin wrapper around the YDLIDAR serial protocol."""

    def __init__(self, port_name: str, baud_rate: int, node_logger=None) -> None:
        self.port_name = port_name
        self.baud_rate = baud_rate
        self._logger = node_logger if node_logger is not None else logger
        if node_logger is None:
            logging.basicConfig(level=logging.INFO)
        self._ser = serial.Serial(self.port_name, self.baud_rate)
        self._ser.flushInput()
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        self.incomming_info: List[str] = ['ODO', 'VW', 'POSE', 'ACCL', 'GYRO']
        self._vel = [0.0, 0.0]
        self._enc = [0.0, 0.0]
        self._wodom = [0.0, 0.0]
        self._rpm = [0.0, 0.0]
        self._wvel = [0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._imu = [0.0, 0.0, 0.0]

    def set_periodic_info(self, interval_ms: int) -> None:
        for idx, each in enumerate(self.incomming_info):
            self.write_port(f"$cREGI,{idx},{each}")
            sleep(0.02)

        self.write_port(f"$cPERI,{interval_ms}")
        sleep(0.01)
        self.write_periodic_query_enable(1)

    def get_port_state(self) -> bool:
        return self._ser.is_open

    def read_port(self) -> bytes:
        if self.get_port_state():
            return self._ser.readline()
        return b''

    def close_port(self) -> None:
        self._logger.info("Port close")
        self._ser.close()

    def read_packet(self) -> None:
        if not self.get_port_state():
            return

        raw_line = self._ser.readline()
        whole_packet = raw_line.split(b'\r')[0].decode("utf-8").strip()
        if not whole_packet:
            return

        self._logger.debug(f"Packet received: {whole_packet}")
        packet = whole_packet.split(',')
        try:
            header_tokens = packet[0].split('#', 1)
            if len(header_tokens) < 2:
                return
            header = header_tokens[1].lstrip('Q')
            if header.startswith('VW'):
                self._vel = [float(packet[1]), float(packet[2])]
            elif header.startswith('ENCOD'):
                self._enc = [int(packet[1]), int(packet[2])]
            elif header.startswith('ODO'):
                self._wodom = [float(packet[1]), float(packet[2])]
            elif header.startswith('RPM'):
                self._rpm = [int(packet[1]), int(packet[2])]
            elif header.startswith('DIFFV'):
                self._wvel = [int(packet[1]), int(packet[2])]
            elif header.startswith('GYRO'):
                self._gyro = [float(packet[1]), float(packet[2]), float(packet[3])]
            elif header.startswith('POSE'):
                self._imu = [float(packet[1]), float(packet[2]), float(packet[3])]
        except (IndexError, ValueError) as exc:
            self._logger.debug(f"Failed to parse packet '{whole_packet}': {exc}")

    def update_battery_state(self) -> None:
        self.write_port("$qBAT")
        sleep(0.01)

    def get_base_velocity(self):
        return self._vel

    def get_wheel_encoder(self):
        return self._enc

    def get_wheel_odom(self):
        return self._wodom

    def get_wheel_rpm(self):
        return self._rpm

    def get_wheel_velocity(self):
        return self._wvel

    def write_periodic_query_enable(self, param: int) -> None:
        self.write_port(f"$cPEEN,{param}")
        sleep(0.05)

    def write_odometry_reset(self) -> None:
        self.write_port("$cODO,0")
        sleep(0.05)

    def write_base_velocity(self, lin_vel: float, ang_vel: float) -> None:
        self.write_port(f'$CVW,{lin_vel:.0f},{ang_vel:.0f}')

    def write_wheel_velocity(self, wheel_l_lin_vel: float, wheel_r_lin_vel: float) -> None:
        self.write_port(f'$CDIFFV,{wheel_l_lin_vel:.0f},{wheel_r_lin_vel:.0f}')

    def write_port(self, buffer: str) -> None:
        if self.get_port_state():
            self._ser.write((buffer + "\r\n").encode())
