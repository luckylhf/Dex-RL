import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
import yaml

from rl_control_node import XMIGCSControlNode
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class XMIGCSControlNode_sim(XMIGCSControlNode):
    def __init__(self, debug=False):
        super().__init__(debug, sim=True)
        print("rewrite sim")
        # Simulation mode is selected before the interface or control thread starts.

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        executor = SingleThreadedExecutor()
        node = XMIGCSControlNode_sim(debug=False)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if "node" in locals() and node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
