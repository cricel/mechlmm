from dotenv import load_dotenv
import os
import psycopg2

import signal
import sys

class DBConnector:
    def __init__(self):
        signal.signal(signal.SIGINT, self.cleanup)

        load_dotenv()

        self.db_conn = psycopg2.connect(
            host = os.getenv('DB_HOSTNAME'),
            database = os.getenv('DB_NAME'),
            user = os.getenv('DB_USERNAME'),
            password = os.getenv('DB_PASSWORD'),
            port = os.getenv('DB_PORT')
        )

        self.db_cur = self.db_conn.cursor()

        self.debugging_init_table()
        self.init_table()

    #region Init

    def init_table(self):
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

    def debugging_init_table(self):
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
    
    def cleanup(self, signal_received, frame):
        print("clean up")
        self.db_cur.close()
        self.db_conn.close()

        sys.exit(0)
    #endregion

    #region Tools

    def insert_daily_conversion_buffer(self, _name, _dialogue):
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

    def select_delete_daily_conversion_buffer(self):
        conversion_buffer = []

        self.db_cur.execute(
            """
                SELECT * FROM daily_conversion_buffer
            """
        )
        
        for row in self.db_cur.fetchall():
            conversion_buffer.append({
                "name": row[1],
                "dialogue": row[2]
            })
        
        self.db_cur.execute(
            """
                DELETE FROM daily_conversion_buffer;
            """
        )
        self.db_conn.commit()

        return conversion_buffer
    
    def insert_from_buffer_to_conversion(self):
        self.db_cur.execute(
            """
                INSERT INTO daily_conversion (name, dialogue, timestamp)
                SELECT name, dialogue, timestamp
                FROM daily_conversion_buffer
            """
        )
        self.db_conn.commit()

    def select_chat_history(self):
        chat_history = []

        self.db_cur.execute(
            """
                SELECT * FROM chat_history
            """
        )
        
        for row in self.db_cur.fetchall():
            chat_history.append({
                "role": row[2],
                "parts": [{ "text": row[3] }]
            })
        
        return chat_history
    
    def insert_chat_history(self, _session_id, _role, _parts):
        self.db_cur.execute(
            """
                INSERT INTO chat_history (session_id, role, parts) VALUES
                (%s, %s, %s)
            """, 
            (
                _session_id, _role, _parts
            )
        )
                                
        self.db_conn.commit()

    def insert_ai_tasks(self, _name, _task, _date, _location, _status):
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
    
if __name__ == "__main__":
    db_connector = DBConnector()