#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse

from servo_module import ServoBoom


class Main:
    def __init__(self, joint_name, node_id):
        dirname = os.path.dirname(__file__)

        self.servo = ServoBoom(node_id, os.path.join(dirname, 'resources/CopleyAmp.eds'))
        self.servo.send_callback = self.servo_callback
        
        self.active = False


    def run(self):
        if self.servo.open() != "up":
            print("ERR: Open servo")

        self.active = True

        while self.active:
            pass
            
    def servo_callback(self):
        pass

    def close(self):
        self.active = False
        self.servo.close()
        self.client.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ServoNode')
    parser.add_argument('--name', type=str,
                        help='Name of this joint')
    parser.add_argument('--nodeid', type=int,
                        help='CAN Node ID')
    args = parser.parse_args()

    bot = Main(args.name, args.nodeid)

    try:
        bot.run()
    except KeyboardInterrupt:
        bot.close()
        print("Exit\n")
