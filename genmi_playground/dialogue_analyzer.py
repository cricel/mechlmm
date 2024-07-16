from gemini_core import GeminiCore 

import psycopg2
import json

class DialogueAnalyzer:
    def __init__(self):
        self.gemini_core = GeminiCore()

        self.conversion_buffer = []
        self.full_conversion = []

        self.init_db()

    
    def init_db(self):
        self.db_conn = psycopg2.connect(
            host = "localhost",
            database = "gemini_db",
            user = "postgres",
            password = "qwepoi123",
            port = 5432
        )

        self.db_cur = self.db_conn.cursor()

        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS daily_conversion;
            """
        )
        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS daily_conversion_buffer;
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
        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS daily_conversion_buffer 
                (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(255),
                    dialogue TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """
        )

        self.db_conn.commit()
        # self.db_cur.close()
        # self.db_conn.close()

    def post_conversion_buffer_db(self, _name, _dialogue):
        self.db_cur.execute(
            """
                INSERT INTO daily_conversion_buffer (name, dialogue) VALUES
                (%s, %s)
            """, 
            (
                _name, _dialogue
            )
        )
                                
        self.db_conn.commit()

    def load_conversion_buffer_db(self):
        self.db_cur.execute(
            """
                SELECT * FROM daily_conversion_buffer
            """
        )
        
        for row in self.db_cur.fetchall():
            self.conversion_buffer.append({
                "name": row[1],
                "dialogue": row[2]
            })
        
        self.db_cur.execute(
            """
                DELETE FROM daily_conversion_buffer;
            """
        )
        self.db_conn.commit()

    def testing_flow_conversion_txt(self, _path):
        with open(_path, 'r') as file:
            dialogue_counter = 0

            for line in file:
                line = line.strip()
                if ':' in line and len(line.split(':')) == 2:
                    dialogue_counter += 1

                    key, value = line.split(':')

                    self.post_conversion_buffer_db(key.strip(), value.strip())

                    if(dialogue_counter % 10 == 0):
                        self.load_conversion_buffer_db()
                        print(self.gemini_core.ask((
                            """
                                give a list of actionable item for everyone base on the conversion dialogue below, and structure it as when, who, where, what, and status (task status), if no action item found, then simple say "Not Found"

                                %s
                            """, (json.dumps(self.conversion_buffer)))
                        )
                        )
                        print("-=-=-=-=-=-")

                        self.conversion_buffer = []
                        


if __name__ == "__main__":
    dialogue_analyzer = DialogueAnalyzer()
    dialogue_analyzer.testing_flow_conversion_txt("../data/raw_conversion.txt")