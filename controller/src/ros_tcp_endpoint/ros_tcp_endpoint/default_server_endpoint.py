#!/usr/bin/env python

import rclpy

from ros_tcp_endpoint import TcpServer


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    try:
        tcp_server.start()
        tcp_server.setup_executor()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tcp_server.destroy_nodes()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
