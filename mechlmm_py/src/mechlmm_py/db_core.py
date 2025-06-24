import sqlite3
import os
import json
from datetime import datetime

class DB_Core:
    def __init__(self, reset=True, db_path=None):
        if db_path is None:
            db_path = os.path.join(os.path.dirname(__file__), 'mechlmm.db')
        self.init_db(reset, db_path)

    def init_db(self, _reset, db_path):
        self.db_conn = sqlite3.connect(db_path)
        self.db_cur = self.db_conn.cursor()

        if _reset:
            self.db_cur.execute("DROP TABLE IF EXISTS objects_map;")
            self.db_cur.execute("DROP TABLE IF EXISTS video_summaries;")
            self.db_cur.execute("DROP TABLE IF EXISTS data_log;")
            self.db_cur.execute("DROP TABLE IF EXISTS objects_list;")
            self.db_conn.commit()

            self.db_cur.execute(
                '''
                CREATE TABLE IF NOT EXISTS objects_map (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT UNIQUE,
                    features TEXT,
                    reference_videos TEXT
                );
                '''
            )

            self.db_cur.execute(
                '''
                CREATE TABLE IF NOT EXISTS video_summaries (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    file_name TEXT UNIQUE,
                    start_time INTEGER NOT NULL,
                    end_time INTEGER NOT NULL,
                    summary TEXT
                );
                '''
            )

            self.db_cur.execute(
                '''
                CREATE TABLE IF NOT EXISTS data_log (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    topic_name TEXT NOT NULL,
                    data TEXT NOT NULL
                );
                '''
            )

            self.db_cur.execute(
                '''
                CREATE TABLE IF NOT EXISTS objects_list (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    pose TEXT NOT NULL
                );
                '''
            )
            self.db_conn.commit()

    def post_test_data_db(self, _name, _pose):
        insert_query = """
            INSERT INTO objects_list (name, pose)
            VALUES (?, ?)
        """
        self.db_cur.execute(insert_query, (_name, _pose))
        self.db_conn.commit()

    def get_test_data(self, _name):
        select_query = """
            SELECT *
            FROM objects_list
            WHERE name = ?
        """
        self.db_cur.execute(select_query, (_name,))
        result = self.db_cur.fetchone()
        if result:
            col_names = [desc[0] for desc in self.db_cur.description]
            return dict(zip(col_names, result))
        else:
            return None

    def get_table(self, _table_name):
        select_query = f"""
        SELECT * FROM {_table_name}
        ORDER BY id DESC
        LIMIT 100
        """
        self.db_cur.execute(select_query)
        result = self.db_cur.fetchall()
        if result:
            col_names = [desc[0] for desc in self.db_cur.description]
            result_dicts = [dict(zip(col_names, row)) for row in result]
            return result_dicts
        else:
            return None

    def post_data_log_db(self, _timestamp, _topic_name, _data):
        insert_query = """
            INSERT INTO data_log (timestamp, topic_name, data)
            VALUES (?, ?, ?)
        """
        self.db_cur.execute(insert_query, (_timestamp, _topic_name, _data))
        self.db_conn.commit()

    def post_objects_map_db(self, _name, _features, _reference_videos):
        # Store features and reference_videos as JSON strings
        features_json = json.dumps(_features)
        reference_videos_json = json.dumps(_reference_videos)
        insert_query = """
            INSERT INTO objects_map (name, features, reference_videos)
            VALUES (?, ?, ?)
            ON CONFLICT(name) DO UPDATE SET
                features=excluded.features,
                reference_videos=excluded.reference_videos
        """
        try:
            self.db_cur.execute(insert_query, (_name, features_json, reference_videos_json))
        except sqlite3.OperationalError:
            # For older SQLite versions, fallback to manual upsert
            self.db_cur.execute("SELECT id FROM objects_map WHERE name = ?", (_name,))
            if self.db_cur.fetchone():
                self.db_cur.execute(
                    "UPDATE objects_map SET features = ?, reference_videos = ? WHERE name = ?",
                    (features_json, reference_videos_json, _name)
                )
            else:
                self.db_cur.execute(
                    "INSERT INTO objects_map (name, features, reference_videos) VALUES (?, ?, ?)",
                    (_name, features_json, reference_videos_json)
                )
        self.db_conn.commit()

    def get_objects_map_record_by_name_db(self, name):
        select_query = """
            SELECT *
            FROM objects_map
            WHERE name = ?
        """
        self.db_cur.execute(select_query, (name,))
        result = self.db_cur.fetchone()
        if result:
            col_names = [desc[0] for desc in self.db_cur.description]
            row_dict = dict(zip(col_names, result))
            # Parse JSON fields
            if row_dict.get('features'):
                row_dict['features'] = json.loads(row_dict['features'])
            if row_dict.get('reference_videos'):
                row_dict['reference_videos'] = json.loads(row_dict['reference_videos'])
            return row_dict
        else:
            return None

    def get_objects_map_name_list_db(self):
        select_query = """
            SELECT name
            FROM objects_map
        """
        self.db_cur.execute(select_query)
        names = [row[0] for row in self.db_cur.fetchall()]
        return names

    def get_video_summary_list_db(self):
        select_query = """
            SELECT *
            FROM video_summaries
        """
        self.db_cur.execute(select_query)
        rows = self.db_cur.fetchall()
        return rows

    def get_video_summary_record_by_name_db(self, _filename):
        select_query = """
            SELECT *
            FROM video_summaries
            WHERE file_name = ?
        """
        self.db_cur.execute(select_query, (_filename,))
        result = self.db_cur.fetchone()
        if result:
            col_names = [desc[0] for desc in self.db_cur.description]
            return dict(zip(col_names, result))
        else:
            return None

    def get_video_summary_name_list_db(self):
        select_query = """
            SELECT file_name
            FROM video_summaries
        """
        self.db_cur.execute(select_query)
        names = [row[0] for row in self.db_cur.fetchall()]
        return names

    def post_video_record_db(self, _file_name, _start_time, _end_time):
        insert_query = """
            INSERT INTO video_summaries (file_name, start_time, end_time)
            VALUES (?, ?, ?);
        """
        self.db_cur.execute(insert_query, (_file_name, _start_time, _end_time))
        self.db_conn.commit()

    def post_video_summary_db(self, _file_name, _summary):
        update_query = """
            UPDATE video_summaries
            SET summary = ?
            WHERE file_name = ?
        """
        self.db_cur.execute(update_query, (_summary, _file_name))
        self.db_conn.commit()

if __name__ == "__main__":
    sqlite_core = DB_Core()
    # Example usage:
    # sqlite_core.post_test_data_db("coke", json.dumps({"position": {"x": 1, "y": 1, "z": 2}, "angular": {"x": 1, "y": 1, "z": 2}})) 