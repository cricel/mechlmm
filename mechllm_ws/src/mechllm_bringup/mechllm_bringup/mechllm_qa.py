from mechllm_core import MechLLMCore
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    # mechllm_core = MechLLMCore()

    # mechllm_core.chat_text_knowledge("what is in front of the wall")

    try:
        package_name = 'mechllm_bringup'
        package_path = get_package_share_directory(package_name)
        print(f"The path of the package '{package_name}' is: {package_path}")
    except:
        print(f"Package not found.")


if __name__ == '__main__':
    main()