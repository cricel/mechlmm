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
                CREATE TABLE IF NOT EXISTS objects_map (
                    id SERIAL PRIMARY KEY,
                    name VARCHAR(255) UNIQUE,
                    features TEXT[],
                    reference_videos TEXT[][]
                );
            """
        )

        self.db_conn.commit()

    def post_objects_map_db(self, name, features, reference_videos):
        insert_query = """
            INSERT INTO objects_map (name, features, reference_videos) 
            VALUES (%s, %s, %s)
            ON CONFLICT (name) 
            DO UPDATE SET 
                features = EXCLUDED.features,
                reference_videos = EXCLUDED.reference_videos
        """

        self.db_cur.execute(insert_query, (name, features, reference_videos))
        self.db_conn.commit()

    def get_objects_map_features_db(self, name):
        select_query = """
            SELECT features 
            FROM objects_map 
            WHERE name = %s
        """
        self.db_cur.execute(select_query, (name,))
        result = self.db_cur.fetchone()

        if result:
            return result[0]
        else:
            return None
        
    def get_objects_map_record_by_name_db(self, name):
        select_query = """
            SELECT * 
            FROM objects_map 
            WHERE name = %s
        """
        self.db_cur.execute(select_query, (name,))
        result = self.db_cur.fetchone()

        if result:
            col_names = [desc[0] for desc in self.db_cur.description]
            return dict(zip(col_names, result))
        else:
            return None

if __name__ == "__main__":
    postgres_core = PostgresCore()
    postgres_core.post_objects_map_db("test_key", ["test", "test2"], [["test", "0", "30"], ["tes2t", "10", "40"]])