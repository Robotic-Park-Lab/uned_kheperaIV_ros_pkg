#!/usr/bin/python3
## ^^^^^^^^^^^^^^^^^^^^^
## Libs
## ^^^^^^^^^^^^^^^^^^^^^
import sys
import os
import subprocess
import time
import yaml
import rclpy
from subprocess import check_output

from main_ui import *
import matplotlib.pyplot as plt
import numpy as np
from qt_gui.settings import Settings
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from rqt_gui_py.plugin import Plugin
from rqt_plot.plot import Plot
from rqt_plot.plot_widget import PlotWidget
from shell_cmd import ShellCmd


## ^^^^^^^^^^^^^^^^^^^^^
## RQT Classes
## ^^^^^^^^^^^^^^^^^^^^^
# Implementación extraida de rqt_embed_window
class RQT_Panel( QWidget ):
	def __init__(self, plugin):
		QWidget.__init__(self)
		self._command = plugin
		self._window_name = None
		self._external_window_widget = None
		self._process = None
		self._timeout_to_window_discovery = 10.0

		# Add widget to the user interface
		self.add_external_window_widget()

	def add_external_window_widget(self):
        # The command is prepended with exec so it becomes the shell executing it
        # So it effectively has the PID we will look for the window ID
		self._process = subprocess.Popen([self._command])
		window_id = self.wait_for_window_id()

		# Get window ID from PID, we must wait for it to appear
		# self.window_id = self.wait_for_window_id(pid=self._process.get_pid(), window_name=self._window_name, timeout=self._timeout_to_window_discovery)
		if window_id is None:
			self._process.kill()
			return

        # Create a the window that will contain the program
		window = QWindow.fromWinId(window_id)
        # FramelessWindowHint is necessary for the window to effectively get embedded
		window.setFlags(Qt.FramelessWindowHint)
		widget = QWidget.createWindowContainer(window)

        # Store it for later
		self._external_window_widget = widget

		layout = QVBoxLayout()
		layout.addWidget(self._external_window_widget)
		self.setLayout(layout)

	def close(self):
		self._process.kill()

	def wait_for_window_id(self):
		pid=self._process.pid
		window_id = None
		ini_t = time.time()
		now = time.time()
		while window_id is None and (now - ini_t) < self._timeout_to_window_discovery:
			if pid is not None:
				window_id = self.get_window_id_by_pid(pid)
			else:
				raise RuntimeError("No PID or window_name provided to look for a window on wait_for_window_id")
			time.sleep(0.2)
			now = time.time()
		return window_id

	def get_window_id_by_pid(self, pid):
		p = subprocess.Popen(["xdotool", "search", "--pid", str(pid)], stdout=subprocess.PIPE)
		out = p.communicate()[0]
		output = out.decode("ascii").strip()
		line = output.splitlines()
		p.kill()
		if len(line) >= 2:
			return int(line[1])
		return None


class MainWindow(QMainWindow):
	def __init__(self, node):
		#Qt Stuff..
		super(MainWindow, self).__init__()
		# Load the .ui file made from Qt designer
		uic.loadUi('main.ui', self)

		self.setWindowIcon(QtGui.QIcon(":/figs/LogoRoboticPark.png"))
		self.setWindowTitle("Robotic Park. Khepera IV")

		
		self.steer = RQT_Panel("rqt_robot_steering")
		self.ControlWidget.addTab(self.steer, "Open Loop")

		self.rqt = RQT_Panel("rqt")
		self.rqt_camera = RQT_Panel("rqt")
		self.rqtgraph = RQT_Panel("rqt_graph")
		self.GraphsWidget.addTab(self.rqt, "Plots")
		self.GraphsWidget.addTab(self.rqtgraph, "Graph")
		self.tabWidget.addTab(self.rqt_camera, "Camera")
		

		node.get_logger().warn("Development in progress ...")
		self.Close_action.triggered.connect(self.cerrar)
		self.Close_action.setShortcut("Ctrl+W")

	def cerrar(self):
		rclpy.shutdown()
		self.rqt.close()
		self.rqtgraph.close()
		self.steer.close()
		time.sleep(1)
		p = subprocess.run(["killall","rqt"])
		QApplication.quit()

if __name__ == '__main__':
    # Init ROS2
	rclpy.init()
	p = subprocess.Popen('ros2 node list', shell=True,stdout=subprocess.PIPE)
	out = p.communicate()[0]
	output = out.decode("ascii").split()
	str_match = [s for s in output if "uned_kh_interface" in s]
	if not str_match:
		interface_node = rclpy.create_node('uned_kh_interface')
	else:
		node_name = "uned_kh_interface_aux"+str(len(str_match))
		interface_node = rclpy.create_node(node_name)
	p.kill()
	app = QApplication(sys.argv)
	interfaz = MainWindow(interface_node)
	interfaz.show()
	sys.exit(app.exec_())
