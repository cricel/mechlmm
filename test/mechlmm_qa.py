from mechlmm_py import utilities_core
from termcolor import colored


if __name__ == '__main__':

    while True:
        print(colored("\033[5m======= User Input =======\033[0m", 'red'))

        user_input = input("How can I help you: ")
        
        if user_input.lower() == "exit":
            print("Exiting...")
            break
        else:
            print(utilities_core.rest_post_request(user_input))