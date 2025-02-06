# -*- coding: utf-8 -*-

import can
import canopen
from pathlib import Path
import logging

logging.basicConfig(level=logging.ERROR)


class Servo:
    """
    Basic CANopen-Node for a servo controller

    :param int can_node_id:
        The id of this node
    :param string can_object_dictionary:
        Object dictionary as a path to an .eds file
    """
    def __init__(self, can_node_id, can_object_dictionary):

        self.can_node_id = can_node_id
        self.can_node_file_path = can_object_dictionary

        self.can_network = canopen.Network()
        self.can_network_status = 'down'

        self.servo_node = self.can_network.add_node(can_node_id, str(Path(self.can_node_file_path).resolve()))

        self.tpdo_values = {
            'Actual motor position': 0.0,
            'Actual motor velocity': 0.0,
            'Status word': 0,
        }

        self.active = True

        self.velocity = 0
        self.speed_override = 1.0
        self.control_word = 0x00

        self.send_callback = None

    def open(self):
        """
        Open connection to CANopen network

        :return: The status of the network
        :rtype: string
        """
        pass

    def close(self):
        """
        Close the connection to the network
        """
        self.switch_power(False)
        self.can_network_status = 'down'
        self.can_network.sync.stop()
        self.can_network.disconnect()

    # Speed alters velocity in percent
    def set_speed(self, speed):
        if self.can_network_status is "up":
            self.speed_override = speed / 100

    def set_position(self, position):
        """
        Sends the received position set point over CANopen RPDO, if network is
        connected and up running.

        :param position: set point for position, measured in rad
        :return: none
        """
        pass

    def get_position(self):
        """
        Get actual servo position from last received TPDO, measured in rad.

        :return: actual motor position, measured in rad
        """
        pass

    def set_velocity(self, velocity):
        """
        Sends the received position set point over CANopen RPDO, if network is
        connected and up running.

        :param velocity: set point for velocity, measured in rad/s
        :return: none
        """
        pass

    def get_velocity(self):
        """
        Get actual servo velocity from last received TPDO, measured in rad/s.

        :return: actual motor velocity, measured in rad/s
        """
        pass

    def stop_move(self):
        """
        Stop all movements, decelerate with predefined rate until full halt.

        :return: none
        """
        pass

    def switch_power(self, on=False):
        """
        Switch the output power supply of the drive controller by setting or clearing
        the following Bits:
        ON  : Set 'Switch on', 'Enable Voltage' and 'Enable Operation'
                Supply will be active and break inactive

        OFF : Clear 'Switch on', 'Enable Voltage' and 'Enable Operation'
                Supply will be inactive and break active

        :param on: on or off
        :return: none
        """
        pass

    def on_command(self, cmd, value, write):
        if cmd == 'ACTIVATE':
            if write:
                self.active = bool(value)
                self.switch_power(self.active)
            return True, int(self.active)

        if not self.active:
            return False, -1

        if cmd == 'STOP':
            self.stop_move()
            return True, 0

        if cmd == 'SIM_SPEED':
            if write:
                self.set_speed(value)
                self.set_velocity(self.velocity)
            return True, float(self.speed_override * 100.0)

        elif cmd == 'VELOCITY':
            if write:
                self.set_velocity(value)
            return True, self.get_velocity()

        elif cmd == 'POSITION':
            if write:
                self.set_position(value)
            return True, self.get_position()

        elif cmd == 'POWER':
            self.switch_power(value)
            return True, value

        return False, 0

    def push_status(self):
        if self.tpdo_values.__contains__("Actual motor position"):
            self.send_callback('POSITION', self.get_position())

        if self.tpdo_values.__contains__("Actual motor velocity"):
            self.send_callback('VELOCITY', self.get_velocity())

    def tpdo_callback(self, message):
        for var in message:
            self.tpdo_values[var.name] = var.raw

        self.push_status()
