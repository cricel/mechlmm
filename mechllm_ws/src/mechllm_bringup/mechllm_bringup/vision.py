from vision_core import VisionCore
import rclpy
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    # rclpy.init(args=args)

    # vision_core = VisionCore(True)
    # vision_core.spin()
    # vision_core.destroy_node()

    # rclpy.shutdown()

    vision_core = VisionCore(False)
    vision_core.run()


if __name__ == '__main__':
    main()