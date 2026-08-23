#!/usr/bin/python3

# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Main window of the Khepera IV PyQt interface.

Deliberately basic: no embedded rqt_robot_steering/rqt_plot/rqt_graph
panels. The original code embedded them via xdotool + QWindow.fromWinId()
(X11-only, and xdotool was never a declared dependency) -- same tradeoff
already made for uned_crazyflie_gui in the sibling Crazyflie repo. See the
"Future work" section in README.md for that fuller vision.
"""
import os
import sys

import rclpy
from PyQt5 import uic
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow

from uned_kheperaiv_gui import logo_rc  # noqa: F401 -- registers the :/figs/... Qt resources

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))


class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        uic.loadUi(os.path.join(PACKAGE_DIR, 'main.ui'), self)

        self.setWindowIcon(QIcon(':/figs/LogoRoboticPark.png'))
        self.setWindowTitle('Robotic Park. Khepera IV')

        self.node = node
        self.Close_action.triggered.connect(self.close)
        self.Close_action.setShortcut('Ctrl+W')


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('uned_kh_interface')

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
