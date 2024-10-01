from mechlmm_py import MechLMMCore
from termcolor import colored

from dotenv import load_dotenv
load_dotenv()

if __name__ == '__main__':
    mechlmm_core = MechLMMCore()

    while True:
        print(colored("\033[5m======= User Input =======\033[0m", 'red'))

        user_input = input("How can I help you: ")
        
        if user_input.lower() == "exit":
            print("Exiting...")
            break
        else:
            mechlmm_core.chat_text(user_input)