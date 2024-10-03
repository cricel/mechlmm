from termcolor import colored

class DebugCore:
    def __init__(self):
        # 0: disable all
        # 1: show error
        # 2: show warning + error
        # 3: show info + warning + error
        self.verbose = 3 

    def log_info(self, _msg):
        if(self.verbose == 3):
            print(colored(_msg, 'green'))

    def log_warning(self, _msg):
        if(self.verbose >= 2 and self.verbose <= 3):
            print(colored(_msg, 'yellow'))

    def log_error(self, _msg):
        if(self.verbose >= 1 and self.verbose <= 3):
            print(colored(_msg, 'red'))

    def log_print(self, _msg):
        if(self.verbose >= 0):
            print(colored(_msg, 'white'))
    
    def log_key(self, _msg):
        if(self.verbose >= 0):
            print(colored(_msg, 'blue'))

    def log_flash(self, _msg):
        if(self.verbose >= 0):
            print(colored(f"\033[5m{_msg}\033[0m", 'red'))

if __name__ == '__main__':
    debug_core = DebugCore()
    debug_core.verbose = 0
    debug_core.log_error("error")
    debug_core.log_info("info")
    debug_core.log_warning("warning")
    debug_core.log_print("print")