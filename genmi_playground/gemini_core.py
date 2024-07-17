import google.generativeai as genai
import PIL.Image
import psycopg2

import uuid

class GeminiCore:
    def __init__(self):
        # DB Config
        self.chat_session_id = str(uuid.uuid4())
        self.chat_history = []

        self.init_db()
        self.get_chat_history_db()

        # Gemini Config
        self.house_fns = [self.tasks_json_convertor, self.power_disco_ball, self.start_music, self.dim_lights]

        self.function_handler_dict = {
            "tasks_json_convertor": self.tasks_json_convertor,
            "power_disco_ball": self.power_disco_ball,
            "start_music": self.start_music,
            "dim_lights": self.dim_lights
        }

        self.init_llm()

    def init_llm(self):
        GOOGLE_API_KEY="AIzaSyCSjCUGUydxiSsWmlMLEU_nfqtu70HPGZo"
        genai.configure(api_key=GOOGLE_API_KEY)
        self.model = genai.GenerativeModel(model_name="gemini-1.5-flash", tools=self.house_fns)
        self.chat = self.model.start_chat(history=self.chat_history)
    
    def ask(self, _msg, _img_path = None):
        result = None

        if(_img_path == None):
            result = self.ask_text(_msg)
        elif(_img_path != None):
            result = self.ask_img(_msg, _img_path)

        return result

    def ask_text(self, _msg):
        response = self.chat.send_message(_msg, 
                                          safety_settings={'HARASSMENT':'block_none', 
                                                           'SEXUALLY_EXPLICIT':'block_none',
                                                           'HATE_SPEECH':'block_none',
                                                           'HARASSMENT':'block_none',
                                                           })

        function_responses = {}
        trigger_function_call = False

        for part in response.parts:
            if fn := part.function_call:
                trigger_function_call = True

                args = ", ".join(f"{key}={val}" for key, val in fn.args.items())
                function_responses[fn.name] = self.function_handler_dict[fn.name](**fn.args)
                print(f"{fn.name}({args}) ==> {function_responses[fn.name]}")
                

        if(trigger_function_call):
            response_parts = [
                genai.protos.Part(function_response=genai.protos.FunctionResponse(name=fn, response={"result": val}))
                for fn, val in function_responses.items()
            ]

            try:
                response = self.chat.send_message(response_parts)
            except Exception as e:
                print(f"======================Getting Error on {e}")

        model_result = ""

        for part in response.parts:
            if tx := part.text:
                if(tx.strip() == ""):
                    print(response)
                    model_result = ""
                else:
                    model_result = response.text.strip()
        # if ("text" in part):
        #     print(response.parts)
        #     model_result = response.text

        self.post_chat_history_db("user", _msg)
        self.post_chat_history_db("model", model_result)
            
        return model_result

        
    
    def ask_img(self, _msg, _img_path):
        img = PIL.Image.open(_img_path)
        response = self.model.generate_content([_msg, img], stream=True)
        response.resolve()

        return response.text
    

    #region PostgresDB

    def init_db(self):
        self.db_conn = psycopg2.connect(
            host = "localhost",
            database = "gemini_db",
            user = "postgres",
            password = "qwepoi123",
            port = 5432
        )

        self.db_cur = self.db_conn.cursor()


        ##### TESTING ONLY #####
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS chat_history;
            """
        )
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS ai_tasks;
            """
        )
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS human_tasks;
            """
        )

        self.db_conn.commit()
        ##### TESTING ONLY #####


        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS chat_history 
                (
                    id SERIAL PRIMARY KEY,
                    session_id UUID,
                    role VARCHAR(10),
                    parts TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )

        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS ai_tasks 
                (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(20),
                    task TEXT,
                    date TIMESTAMP,
                    location VARCHAR(100),
                    status VARCHAR(10),
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )

        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS human_tasks 
                (
                    id SERIAL PRIMARY KEY,
                    task TEXT,
                    status VARCHAR(10),
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )
        
        self.db_conn.commit()
        # self.db_cur.close()
        # self.db_conn.close()

    def post_chat_history_db(self, role, parts):
        self.db_cur.execute(
            """
                INSERT INTO chat_history (session_id, role, parts) VALUES
                (%s, %s, %s)
            """, 
            (
                self.chat_session_id, role, parts
            )
        )
                                
        self.db_conn.commit()

    def get_chat_history_db(self):
        self.db_cur.execute(
            """
                SELECT * FROM chat_history
            """
        )
        
        for row in self.db_cur.fetchall():
            self.chat_history.append({
                "role": row[2],
                "parts": [{ "text": row[3] }]
            })

    def post_ai_tasks_db(self, _name, _task, _date, _location, _status):
        if (_date == ""):
            _date = None
        if (_task != "" and _task.lower != "none" ):
            self.db_cur.execute(
                """
                    INSERT INTO ai_tasks (name, task, date, location, status) VALUES
                    (%s, %s, %s, %s, %s)
                """, 
                (
                    _name, _task, _date, _location, _status
                )
            )
                                    
            self.db_conn.commit()

    #endregion


    #region FunctionCalls

    # person = genai.protos.Schema(
    #     type = genai.protos.Type.OBJECT,
    #     properties = {
    #         'name':  genai.protos.Schema(type=genai.protos.Type.STRING),
    #         'task':  genai.protos.Schema(type=genai.protos.Type.STRING),
    #         'date':  genai.protos.Schema(type=genai.protos.Type.STRING),
    #         'location':  genai.protos.Schema(type=genai.protos.Type.STRING),
    #         'status': genai.protos.Schema(type=genai.protos.Type.STRING),
    #         'timestamp': genai.protos.Schema(type=genai.protos.Type.STRING)
    #     },
    #     required=['name', 'task', 'date', 'location', 'status', 'timestamp']
    # )

    # add_to_database = genai.protos.FunctionDeclaration(
    #     name="add_to_database",
    #     description=textwrap.dedent("""\
    #         Adds entities to the database.
    #         """),
    #     parameters=genai.protos.Schema(
    #         type=genai.protos.Type.OBJECT,
    #         properties = {
    #             'people': people,
    #             'places': places,
    #             'things': things,
    #             'relationships': relationships
    #         }
    #     )
    # )

    def tasks_json_convertor(self, name: str, task: str, date: str, location: str, status: str) -> bool:
        """
        Convert given context into actionable task list as JSON format.

        Args:
        name: the name of the person who is responsble for this task.
        task: the detail description of the task. if no task found, then return none
        date: the date and time that this task will take in place, put it in "YYYY-MM-DD HH:MI:SS" format, return empty string if time or date not applied.
        location: the location that this task will take in place, return empty string if not applied.
        status: the status of current task, value can be "done, pending, canceled, invalid, none".
        """
        
        if(task != ""):
            print(f"===I got Task for {name}: {task} at {date} in {location} is {status}")

        # task_list = []
        self.post_ai_tasks_db(name, task, date, location, status)

        return True
    
    def task_status_checker(self, task: str) -> bool:
        """
        Convert given context into actionable task list as JSON format.

        Args:
        name: the name of the person who is responsble for this task.
        task: the detail description of the task.
        date: the date and time that this task will take in place, put it in MM/dd/yyyy HH:mm:ss format, return none if time or date not applied.
        location: the location that this task will take in place, return none if not applied.
        status: the status of current task, value can be "done, pending, canceled, invalid".

        Returns: json string.
        """
        
        task_list = []
        self.post_ai_tasks_db(task_list)

        return True

    def power_disco_ball(self, power: bool) -> bool:
        """Powers the spinning disco ball."""
        print(f"Disco ball is {'spinning!' if power else 'stopped.'}")
        return True


    def start_music(self, energetic: bool, loud: bool, bpm: int) -> str:
        """Play some music matching the specified parameters.

        Args:
        energetic: Whether the music is energetic or not.
        loud: Whether the music is loud or not.
        bpm: The beats per minute of the music.

        Returns: The name of the song being played.
        """
        print(f"Starting music! {energetic=} {loud=}, {bpm=}")
        return "Never gonna give you up."


    def dim_lights(self, brightness: float) -> bool:
        """Dim the lights.

        Args:
        brightness: The brightness of the lights, 0.0 is off, 1.0 is full.
        """
        print(f"Lights are now set to {brightness:.0%}")
        return True

    #endregion

    #region Tools

    def process_raw_conversion_txt_to_db(self, _path):
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS daily_conversion;
            """
        )
        self.db_conn.commit()

        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS daily_conversion 
                (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(255),
                    dialogue TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )
        self.db_conn.commit()

        with open(_path, 'r') as file:
            for line in file:
                line = line.strip()
                if ':' in line and len(line.split(':')) == 2:
                    key, value = line.split(':')

                    self.db_cur.execute(
                        """
                            INSERT INTO daily_conversion (name, dialogue) VALUES
                            (%s, %s);
                        """, 
                        (
                            key.strip(), value.strip()
                        )
                    )
                                
        self.db_conn.commit()

    def process_raw_conversion_txt_to_db(self, _path):
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS daily_conversion;
            """
        )
        self.db_conn.commit()

        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS daily_conversion 
                (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(255),
                    dialogue TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )
        self.db_conn.commit()

        with open(_path, 'r') as file:
            dialogue_counter = 0

            for line in file:
                line = line.strip()
                if ':' in line and len(line.split(':')) == 2:
                    dialogue_counter += 1

                    key, value = line.split(':')

                    self.db_cur.execute(
                        """
                            INSERT INTO daily_conversion (name, dialogue) VALUES
                            (%s, %s);
                        """, 
                        (
                            key.strip(), value.strip()
                        )
                    )
                                
        self.db_conn.commit()


    #endregion

if __name__ == "__main__":
    gemini_core = GeminiCore()
    
    # print(gemini_core.ask("How are you").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("what is my name").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("How many time have I asked you what is my name").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("My name is Shawn").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("Turn off the light").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("Tell me something about this picture", "./test_img.png").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("what is my name").strip())
    # print("--------------------------------------------")
    # print(gemini_core.ask("what is the current volume of the music").strip())

    gemini_core.process_raw_conversion_txt_to_db("../data/raw_conversion.txt")