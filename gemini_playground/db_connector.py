from dotenv import load_dotenv
import os
import psycopg2

class DBConnector:
    def __init__(self):
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
    
    #endregion

    #region Tools



    #endregion
    
if __name__ == "__main__":
    db_connector = DBConnector()