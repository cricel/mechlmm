import psycopg2
import json

class PostgresCore:
    def __init__(self):
        self.init_db()

    def init_db(self):
        self.db_conn = psycopg2.connect(
            host = "localhost",
            database = "mechllm",
            user = "postgres",
            password = "qwepoi123",
            port = 5432
        )

        self.db_cur = self.db_conn.cursor()

        self.db_cur.execute(
            """
                DROP TABLE IF EXISTS objects_map;
            """
        )

        self.db_conn.commit()

        self.db_cur.execute(
            """
                CREATE TABLE IF NOT EXISTS objects_map 
                (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(255)
                );
            """
        )

        self.db_conn.commit()

if __name__ == "__main__":
    postgres_core = PostgresCore()