#!/usr/bin/env python3

import os
import rclpy
import rclpy.node

class PX4GZSIM(rclpy.node.Node):
    def __init__(self):
        super().__init__('px4_gz_sim')

        self.PX4_DIR = os.getenv('PX4_DIR')

        if self.PX4_DIR is not None:
            print(f'The value of PX4_DIR is {self.PX4_DIR}')
        else:
            print('PX4_DIR is not set')
            return

        self.declare_parameter('headless', 0)
        self.declare_parameter('gz_model_name', 'x500')
        self.declare_parameter('px4_autostart_id', 4001)
        self.declare_parameter('instance_id', 0)
        self.declare_parameter('xpos', 0.0)
        self.declare_parameter('ypos', 0.0)
        self.declare_parameter('zpos', 0.0)

        self.run()


    def run(self):
        headless = self.get_parameter('headless').get_parameter_value().integer_value
        self.get_logger().info('headless %s!' % headless)

        gz_model_name = self.get_parameter('gz_model_name').get_parameter_value().string_value
        self.get_logger().info('gz_model_name %s!' % gz_model_name)

        px4_autostart_id = self.get_parameter('px4_autostart_id').get_parameter_value().integer_value
        self.get_logger().info('px4_autostart_id %s!' % px4_autostart_id)

        instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.get_logger().info('instance_id %s!' % instance_id)

        xpos = self.get_parameter('xpos').get_parameter_value().double_value
        self.get_logger().info('xpos %s!' % xpos)

        ypos = self.get_parameter('ypos').get_parameter_value().double_value
        self.get_logger().info('ypos %s!' % ypos)

        zpos = self.get_parameter('zpos').get_parameter_value().double_value
        self.get_logger().info('zpos %s!' % zpos)

        cmd1_str="cd {} && ".format(self.PX4_DIR)
        cmd2_str="PX4_SYS_AUTOSTART={} PX4_GZ_MODEL={} PX4_GZ_MODEL_POSE='{},{},{}' ./build/px4_sitl_default/bin/px4 -i {}".format(px4_autostart_id, gz_model_name, xpos,ypos,zpos, instance_id)
        cmd_str = cmd1_str+cmd2_str
        os.system(cmd_str)


def main():
    rclpy.init()
    node = PX4GZSIM()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
