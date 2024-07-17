from db_connector import DBConnector


class ToolBox:
    def __init__(self):
        self.db_connector = DBConnector()

    def tasks_json_convertor(self, name, task, date, location, status):
        if(task != ""):
            print(f"===I got Task for {name}: {task} at {date} in {location} is {status}")

        self.db_connector.insert_chat_history(name, task, date, location, status)
    

    def task_status_checker(self, task):
        task_list = []
        self.db_connector.insert_chat_history(task_list)
    
    def power_disco_ball(self, power):
        print(f"Disco ball is {'spinning!' if power else 'stopped.'}")
    
    def start_music(self, energetic, loud, bpm):
        print(f"Starting music! {energetic=} {loud=}, {bpm=}")
    
    def dim_lights(self, brightness):
        print(f"Lights are now set to {brightness:.0%}")
    
if __name__ == "__main__":
    tool_box = ToolBox()