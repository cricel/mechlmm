from gemini_core import GeminiCore 
from db_connector import DBConnector

import json

class DialogueAnalyzer:
    def __init__(self):
        self.gemini_core = GeminiCore()
        self.db_connector = DBConnector()

    def testing_flow_conversion_txt(self, _path):
        with open(_path, 'r') as file:
            dialogue_counter = 0

            for line in file:
                line = line.strip()
                if ':' in line and len(line.split(':')) == 2:
                    dialogue_counter += 1

                    key, value = line.split(':')

                    self.db_connector.insert_daily_conversion_buffer(key.strip(), value.strip())

                    if(dialogue_counter % 10 == 0):
                        daily_conversion_buffer = self.db_connector.select_delete_daily_conversion_buffer()

                        print(
                            self.gemini_core.ask((
                                """
                                    Based on the conversation dialogue below, provide a list of specific, actionable tasks that can be executed, such as 'set an alarm for 5 PM' or 'schedule a meeting with Carl.' Each task should be clearly defined and practical. And ignore those are ambiguous and too general. Trigger a function call for each item in the list. If no task found, then dont trigger function call:

                                    %s
                                """, (json.dumps(daily_conversion_buffer)))
                            )
                        )
                        print("-------------- ^^ Task Generated ^^ --------------")

                        self.db_connector.insert_from_buffer_to_conversion()


if __name__ == "__main__":
    dialogue_analyzer = DialogueAnalyzer()
    dialogue_analyzer.testing_flow_conversion_txt("../../resources/test/conversions/raw_conversion.txt")