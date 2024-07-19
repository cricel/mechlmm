import schedule
import time
import psycopg2
from datetime import datetime
from dotenv import load_dotenv
import os

from multiprocessing import Process, Event

class ToDoScheduler:
    def __init__(self, stop_event):
        load_dotenv()
        self.stop_event = stop_event
        self.sleep_time = 5   # 5 seconds
        self.db_params = {
            "host": os.getenv("DB_HOST"),
            "dbname": os.getenv("DB_NAME"),
            "user": os.getenv("DB_USER"),
            "password": os.getenv("DB_PASSWORD"),
            "port": os.getenv("DB_PORT")
        }
        
        print(self.db_params)
        self.setup_db()
    
    def connect_db(self):
        """Connect to the PostgreSQL database."""
        conn = psycopg2.connect(**self.db_params)
        return conn

    def setup_db(self):
        """Set up the database and create the ToDo table if it doesn't exist."""
        conn = self.connect_db()
        cursor = conn.cursor()
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS ToDo (
            id SERIAL PRIMARY KEY,
            time TIMESTAMP NOT NULL,
            event TEXT NOT NULL,
            action TEXT NOT NULL
        )
        ''')
        conn.commit()
        conn.close()

    def read_todos(self):
        """Read all ToDos from the database."""
        conn = self.connect_db()
        cursor = conn.cursor()
        cursor.execute("SELECT id, time, event, action FROM ToDo")
        todos = cursor.fetchall()
        conn.close()
        return todos

    def execute_action(self, action):
        """Execute the action for the ToDo."""
        print(f"Executing action: {action}")

    def check_and_execute_todos(self):
        """Check the ToDo table and execute actions for expired ToDos."""
        todos = self.read_todos()
        now = datetime.now()
        for todo in todos:
            todo_id, todo_time, event, action = todo
            if now >= todo_time:
                print(f"Event: {event} - Time: {todo_time} has expired. Executing action.")
                self.execute_action(action)
                self.delete_todo_by_id(todo_id)

    def delete_todo_by_id(self, todo_id):
        """Delete the ToDo by its ID after executing the action."""
        conn = self.connect_db()
        cursor = conn.cursor()
        cursor.execute("DELETE FROM ToDo WHERE id = %s", (todo_id,))
        conn.commit()
        conn.close()

    def schedule_task(self):
        """Schedule the task to check the ToDo table every minute."""
        schedule.every(1).minute.do(self.check_and_execute_todos)

        while not self.stop_event.is_set():
            schedule.run_pending()
            time.sleep(self.sleep_time)

    def test_connection(self):
        pass





if __name__ == "__main__":
    stop_event = Event()
    schedule = ToDoScheduler(stop_event)
    schedule.test_connection()

    
    