import rclpy
from ament_index_python.packages import get_package_share_directory

import os

from vision_core import VisionCore

def main(args=None):
    try:
        package_name = 'mechllm_bringup'
        package_path = get_package_share_directory(package_name)
        print(f"The path of the package '{package_name}' is: {package_path}")
    except:
        print(f"Package not found.")


    # rclpy.init(args=args)

    # vision_core = VisionCore(True, os.path.join(package_path, "output"))
    # vision_core.spin()
    # vision_core.destroy_node()

    # rclpy.shutdown()

    vision_core = VisionCore(False, os.path.join(package_path, "output"))
    vision_core.run()

if __name__ == '__main__':
    main()