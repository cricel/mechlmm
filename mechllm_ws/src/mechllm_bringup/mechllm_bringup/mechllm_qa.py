from ament_index_python.packages import get_package_share_directory

import os

from mechllm_core import MechLLMCore

def main(args=None):
    try:
        package_name = 'mechllm_bringup'
        package_path = get_package_share_directory(package_name)
        print(f"The path of the package '{package_name}' is: {package_path}")
    except:
        print(f"Package not found.")

    mechllm_core = MechLLMCore(os.path.join(package_path, "output"))
    mechllm_core.chat_text_knowledge("who broke the sofa")

if __name__ == '__main__':
    main()