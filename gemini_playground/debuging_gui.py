from dotenv import load_dotenv
import sys
import os

import psycopg2

from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QPushButton, QGridLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QIcon



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        load_dotenv()

        self.setWindowTitle("DB Debuger")
        self.setGeometry(100, 100, 800, 600)

        self.setWindowIcon(QIcon("./debuger_icon.png"))
        # self.setWindowIcon(QIcon("./debuger_icon.icns"))

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QGridLayout(self.central_widget)

        self.table_widget1 = QTableWidget()
        self.table_widget2 = QTableWidget()
        self.table_widget3 = QTableWidget()
        self.table_widget4 = QTableWidget()

        self.layout.addWidget(self.table_widget1, 0, 0)
        self.layout.addWidget(self.table_widget2, 0, 1)
        self.layout.addWidget(self.table_widget3, 1, 0)
        self.layout.addWidget(self.table_widget4, 1, 1)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_tables)
        self.layout.addWidget(self.refresh_button, 2, 0, 1, 2)

        self.load_data(self.table_widget1, "chat_history")
        self.load_data(self.table_widget2, "ai_tasks")
        self.load_data(self.table_widget3, "human_tasks")
        self.load_data(self.table_widget4, "daily_conversion_buffer")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_tables)
        self.timer.start(1000)

    def load_data(self, table_widget, table_name):
        conn = psycopg2.connect(
            host = "localhost",
            database = "gemini_db",
            user = "postgres",
            password = "qwepoi123",
            port = 5432
        )
        cursor = conn.cursor()

        cursor.execute(f"SELECT * FROM {table_name}")

        rows = cursor.fetchall()

        colnames = [desc[0] for desc in cursor.description]

        table_widget.setRowCount(len(rows))
        table_widget.setColumnCount(len(colnames))

        table_widget.setHorizontalHeaderLabels(colnames)

        for i, row in enumerate(rows):
            for j, col in enumerate(row):
                table_widget.setItem(i, j, QTableWidgetItem(str(col)))

        cursor.close()
        conn.close()

    def refresh_tables(self):
        self.load_data(self.table_widget1, "chat_history")
        self.load_data(self.table_widget2, "ai_tasks")
        self.load_data(self.table_widget3, "human_tasks")
        self.load_data(self.table_widget4, "daily_conversion_buffer")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
