#!/usr/bin/env python

import os
import rospy
import rospkg
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QPushButton, QVBoxLayout, QWidget, QLabel
from python_qt_binding.QtCore import QTimer
from std_msgs.msg import String
from std_msgs.msg import Int16

from mechlmm_py import TTS_Core, utilities_core

class HardwareSimulator(Plugin):

    def __init__(self, context):
        super(HardwareSimulator, self).__init__(context)
        self.setObjectName('HardwareSimulator')

        ui_file = os.path.join(rospkg.RosPack().get_path('mechlmm_gui'), 'resource', 'hardware_simulator.ui')

        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self

        loadUi(ui_file, self._widget)

        # Set layout to widget
        self._widget.setLayout(self._layout)
        context.add_widget(self._widget)

        self.battery_pub = rospy.Publisher('/battery', Int16, queue_size=10)

        self.battery_value_slider = self._widget.findChild(QWidget, 'battery_value_slider')
        self.battery_label = self._widget.findChild(QLabel, 'battery_value_label')

        self.battery_value_slider.valueChanged.connect(self.battery_value_callback)

        self.flash_timer = QTimer()
        self.flash_timer.setInterval(2000)
        self.flash_timer.timeout.connect(self.timer_callback)
        self.flash_timer.start()

        self.tts_core = TTS_Core()
        self.test_counter = 0

    def timer_callback(self):
        battery_value = self.battery_value_slider.value()

        self.battery_value_slider.setValue(battery_value - 1)

        self.battery_pub.publish(battery_value)

    def battery_value_callback(self):
        battery_value = self.battery_value_slider.value()

        self.battery_pub.publish(battery_value)

        if(battery_value < 30):
            self.battery_label.setStyleSheet("color: red;")
            if (self.test_counter == 0):
                self.test_counter = 1
                query = f"""
                    question': "you are the robot ai, tell me what to do if the data provided is abnormal, otherwise, saying everything is fine

                    ["battery": {battery_value}]

                    """
                
                data = {
                    "question": query
                }
                response = utilities_core.rest_post_request(data, 'http://192.168.1.134:5001/mechlmm/chat/data')
                self.tts_core.tts_play(response["result"])

        elif(battery_value < 50 and battery_value > 30):
            self.battery_label.setStyleSheet("color: yellow;")
        else:
            self.battery_label.setStyleSheet("color: green;")

    def shutdown_plugin(self):
        # Unregister all publishers and subscribers
        self.battery_pub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
