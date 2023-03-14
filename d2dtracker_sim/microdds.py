#!/usr/bin/env python3

import os
import rclpy
import rclpy.node

class XRCEAGENT(rclpy.node.Node):
    def __init__(self):
        super().__init__('microxrceagent_node')

        self.declare_parameter('port', 8888)

        self.run()


    def run(self):
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info('port %s!' % port)

        
        cmd_str = "MicroXRCEAgent udp4 -p {}".format(port)
        os.system(cmd_str)


def main():
    rclpy.init()
    node = XRCEAGENT()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
