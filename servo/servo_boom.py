# -*- coding: utf-8 -*-

import can
import logging
import numpy as np
from .servo import Servo

logging.basicConfig(level=logging.ERROR)


class ServoBoom(Servo):

    ENC_PER_REV = 262144
    GEAR_RATIO = 160

    """
    Basic CANopen-Node for a servo controller

    :param int can_node_id:
        The id of this node
    :param string can_object_dictionary:
        Object dictionary as a path to an .eds file
    """
    def __init__(self, can_node_id, can_object_dictionary):
        super().__init__(can_node_id, can_object_dictionary)

        # initialise control word specific to driver
        self.control_word = 0x20

    def open(self):
        """
        Open connection to CANopen network

        :return: The status of the network
        :rtype: string
        """
        try:
            self.can_network.connect(bustype='kvaser', channel=0, bitrate=1000000)
            self.can_network_status = 'up'
        except can.CanError:
            self.can_network.disconnect()
            self.can_network_status = 'error'
            return self.can_network_status

        # Read current PDO configuration
        self.servo_node.tpdo.read()
        self.servo_node.rpdo.read()

        self.servo_node.tpdo[1].trans_type = 2
        self.servo_node.tpdo[1].enabled = True
        self.servo_node.tpdo[2].trans_type = 2
        self.servo_node.tpdo[2].enabled = True
        self.servo_node.tpdo[3].trans_type = 2
        self.servo_node.tpdo[3].enabled = True
        self.servo_node.tpdo[4].trans_type = 2
        self.servo_node.tpdo[4].enabled = True

        # Reassign RPDO 4 to needed velocity target (Profile Target Velocity =!= Target Velocity) !!!
        self.servo_node.rpdo[4].clear()
        # COB_ID has to be rewritten for valid configuration !
        self.servo_node.rpdo[4].cob_id = 0x500 + self.can_node_id
        self.servo_node.rpdo[4].add_variable('Control word')
        self.servo_node.rpdo[4].add_variable('Profile target velocity')
        self.servo_node.rpdo[4].enable = True

        # Save new configuration (node must be in pre-operational)
        self.servo_node.nmt.state = 'PRE-OPERATIONAL'
        self.servo_node.tpdo.save()
        self.servo_node.rpdo.save()
        self.servo_node.nmt.state = 'OPERATIONAL'

        self.servo_node.tpdo[1].add_callback(self.tpdo_callback)
        self.servo_node.tpdo[2].add_callback(self.tpdo_callback)
        self.servo_node.tpdo[3].add_callback(self.tpdo_callback)
        self.servo_node.tpdo[4].add_callback(self.tpdo_callback)

        # Movement configuration
        self.servo_node.sdo['Modes of operation'].write(1)
        self.servo_node.sdo['Motion profile type'].write(0)

        self.servo_node.sdo['Profile acceleration'].write(500000)
        self.servo_node.sdo['Profile deceleration'].write(500000)
        self.set_velocity(self.velocity)

        self.can_network.sync.start(0.1)

        return self.can_network_status

    def set_position(self, position):
        """
        Sends the received position set point over CANopen RPDO, if network is
        connected and up running.

        :param position: set point for position, measured in rad
        :return: none
        """
        if self.can_network_status is "up":
            self.control_word |= 0x04
            self.servo_node.rpdo[3]['Control word'].write(self.control_word | 0x10, fmt='raw')
            self.servo_node.rpdo[3]['Profile target position'].write(
                int(ServoBoom.GEAR_RATIO * ServoBoom.ENC_PER_REV * position / (np.pi * 2)))
            self.servo_node.rpdo[3].transmit()

            self.servo_node.rpdo[1]['Control word'].write(self.control_word, fmt='raw')
            self.servo_node.rpdo[1].transmit()

    def get_position(self):
        """
        Get actual servo position from last received TPDO, measured in rad.

        :return: actual motor position, measured in rad
        """
        return self.tpdo_values['Actual motor position'] * (2 * np.pi) / ServoBoom.ENC_PER_REV / ServoBoom.GEAR_RATIO

    def set_velocity(self, velocity):
        """
        Sends the received position set point over CANopen RPDO, if network is
        connected and up running.

        :param velocity: set point for velocity, measured in rad/s
        :return: none
        """
        if self.can_network_status is "up":
            self.velocity = velocity
            v = int((self.speed_override * ServoBoom.GEAR_RATIO * ServoBoom.ENC_PER_REV * self.velocity / (np.pi * 2)))

            self.servo_node.rpdo[4]['Control word'].write(self.control_word | 0x10, fmt='raw')
            self.servo_node.rpdo[4]['Profile target velocity'].write(v)
            self.servo_node.rpdo[4].transmit()

            self.servo_node.rpdo[1]['Control word'].write(self.control_word, fmt='raw')
            self.servo_node.rpdo[1].transmit()

    def get_velocity(self):
        """
        Get actual servo velocity from last received TPDO, measured in rad/s.

        :return: actual motor velocity, measured in rad/s
        """
        return self.tpdo_values['Actual motor velocity'] * (2 * np.pi) / ServoBoom.ENC_PER_REV / ServoBoom.GEAR_RATIO

    def stop_move(self):
        """
        Stop all movements, decelerate with predefined rate until full halt.

        :return: none
        """
        if self.can_network_status is "up":
            self.control_word &= ~0x04
            self.servo_node.rpdo[1]['Control word'].write(self.control_word, fmt='raw')
            self.servo_node.rpdo[1].transmit()

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
        if self.can_network_status is "up":
            if on:
                self.control_word |= 0x0B
            else:
                self.control_word &= ~0x08
            self.servo_node.rpdo[1]['Control word'].write(self.control_word, fmt='raw')
            self.servo_node.rpdo[1].transmit()
