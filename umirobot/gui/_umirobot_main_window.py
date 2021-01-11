"""
Copyright (C) 2020-2021 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import os

if os.name == 'nt':
    import ctypes
    import qdarkstyle

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtNetwork import QUdpSocket, QHostAddress

from umirobot.shared_memory import UMIRobotSharedMemoryReceiver
from umirobot.gui._umirobot_main_window_ui import Ui_MainWindow


# Regenerate UI with 'pyuic5 umirobot_main_window.ui -o _umirobot_main_window_ui.py'
class UMIRobotMainWindow(QMainWindow):
    def __init__(self, umi_robot_shared_memory_receiver):
        super(UMIRobotMainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.statusBar().showMessage("Copyright (c) 2020-2021 Murilo M. Marinho (www.murilomarinho.info)")

        self.umi_robot_shared_memory_receiver = umi_robot_shared_memory_receiver
        self.is_open = False
        self.qd = [0, 0, 0, 0, 0, 0]
        self.q_min = -60
        self.q_max = 60

        self.pot_min = [None] * 6
        self.pot_max = [None] * 6
        self.potentiometer_calibration_difference = 3.0

        self.potentiometer_qd = [None] * 6
        self.socket_qd = [None] * 6

        # TODO read socket info from config file
        self.socket_ip = '127.0.0.1'
        self.socket_port = 2222
        self.socket = None
        self.ui.lineedit_teleoperation_socket.setText(str(self.socket_ip + ":" + str(self.socket_port)))
        self._connect_socket()

        # Fix robot figure not showing
        # https://stackoverflow.com/questions/33454555/bind-relative-image-path-to-py-file
        path = os.path.dirname(os.path.abspath(__file__))
        self.ui.label_image.setPixmap(QPixmap(os.path.join(path, 'umirobot.png')))
        self.setWindowIcon(QIcon(os.path.join(path, 'icon.png')))

        self.timer_ = QTimer()
        self.timer_.timeout.connect(self._timer_callback)
        self.timer_.start(1)

        # Increase buttons
        self.ui.button_increase_q_0.clicked.connect(self._button_increase_q_0_clicked_callback)
        self.ui.button_increase_q_1.clicked.connect(self._button_increase_q_1_clicked_callback)
        self.ui.button_increase_q_2.clicked.connect(self._button_increase_q_2_clicked_callback)
        self.ui.button_increase_q_3.clicked.connect(self._button_increase_q_3_clicked_callback)
        self.ui.button_increase_q_4.clicked.connect(self._button_increase_q_4_clicked_callback)
        self.ui.button_increase_q_5.clicked.connect(self._button_increase_q_5_clicked_callback)

        # Decrease buttons
        self.ui.button_decrease_q_0.clicked.connect(self._button_decrease_q_0_clicked_callback)
        self.ui.button_decrease_q_1.clicked.connect(self._button_decrease_q_1_clicked_callback)
        self.ui.button_decrease_q_2.clicked.connect(self._button_decrease_q_2_clicked_callback)
        self.ui.button_decrease_q_3.clicked.connect(self._button_decrease_q_3_clicked_callback)
        self.ui.button_decrease_q_4.clicked.connect(self._button_decrease_q_4_clicked_callback)
        self.ui.button_decrease_q_5.clicked.connect(self._button_decrease_q_5_clicked_callback)

        # Control
        self.ui.checkbox_jointcontrol.clicked.connect(self._checkbox_manual_control_clicked_callback)
        self.ui.slider_manual_control.valueChanged.connect(self._slider_manual_control_value_changed_callback)
        self.ui.label_manual_control_angle_change.setText(
            "Angle change per click " + str(self.ui.slider_manual_control.value()))

        self.ui.checkbox_teleoperation.clicked.connect(self._checkbox_teleoperation_clicked_callback)

        self.ui.checkbox_potentiometer_cal.clicked.connect(self._checkbox_potentiometer_cal_clicked_callback)

        # Connection related buttons
        self.ui.button_connect_port.clicked.connect(self._button_connect_port_clicked_callback)
        self.ui.button_refresh_port.clicked.connect(self._button_refresh_port_clicked_callback)
        self._button_refresh_port_clicked_callback()

    def log(self, msg):
        self.ui.textedit_status.append(msg)

    def _connect_socket(self):
        self.socket = QUdpSocket()
        self.socket.bind(QHostAddress(self.socket_ip), self.socket_port)
        self.socket.readyRead.connect(self._socket_ready_read_callback)

    def _socket_ready_read_callback(self):
        # TODO Read datagram
        pass

    def _get_manual_control_increment(self):
        return self.ui.slider_manual_control.value()

    def _slider_manual_control_value_changed_callback(self):
        self.ui.label_manual_control_angle_change.setText(
            "Angle change per click " + str(self.ui.slider_manual_control.value()))

    def _button_increase_q_0_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[0] = max(self.q_min, min(self.qd[0] + self._get_manual_control_increment(), self.q_max))

    def _button_increase_q_1_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[1] = max(self.q_min, min(self.qd[1] + self._get_manual_control_increment(), self.q_max))

    def _button_increase_q_2_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[2] = max(self.q_min, min(self.qd[2] + self._get_manual_control_increment(), self.q_max))

    def _button_increase_q_3_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[3] = max(self.q_min, min(self.qd[3] + self._get_manual_control_increment(), self.q_max))

    def _button_increase_q_4_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[4] = max(self.q_min, min(self.qd[4] + self._get_manual_control_increment(), self.q_max))

    def _button_increase_q_5_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[5] = max(self.q_min, min(self.qd[5] + self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_0_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[0] = max(self.q_min, min(self.qd[0] - self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_1_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[1] = max(self.q_min, min(self.qd[1] - self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_2_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[2] = max(self.q_min, min(self.qd[2] - self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_3_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[3] = max(self.q_min, min(self.qd[3] - self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_4_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[4] = max(self.q_min, min(self.qd[4] - self._get_manual_control_increment(), self.q_max))

    def _button_decrease_q_5_clicked_callback(self):
        if self._get_manual_control_state():
            self.qd[5] = max(self.q_min, min(self.qd[5] - self._get_manual_control_increment(), self.q_max))

    def _get_manual_control_state(self):
        return self.ui.checkbox_jointcontrol.isChecked()

    def _get_teleoperation_state(self):
        return self.ui.checkbox_teleoperation.isChecked()

    def _checkbox_manual_control_clicked_callback(self):
        if self.ui.checkbox_jointcontrol.isChecked():
            self.log("Info::Switched mode to manual control.")
            self.ui.checkbox_teleoperation.setChecked(False)
            self.ui.checkbox_potentiometer_cal.setChecked(False)

    def _checkbox_teleoperation_clicked_callback(self):
        if self.ui.checkbox_teleoperation.isChecked():
            self.log("Info::Switched mode to teleoperation.")
            self.ui.checkbox_jointcontrol.setChecked(False)
            self.ui.checkbox_potentiometer_cal.setChecked(False)

    def _checkbox_potentiometer_cal_clicked_callback(self):
        if self.ui.checkbox_potentiometer_cal.isChecked():
            self.log("Info::Switched mode to potentiometer calibration.")
            self.ui.checkbox_jointcontrol.setChecked(False)
            self.ui.checkbox_teleoperation.setChecked(False)

    def _timer_callback(self):
        current_is_open = self.umi_robot_shared_memory_receiver.is_open()
        if current_is_open is None:
            self.log("Warning::Error reading shared memory. current_is_open = {}".format(current_is_open))
            return

        if self.is_open and not current_is_open:
            self.log("Warning::Connection lost to {}.".format(self.umi_robot_shared_memory_receiver.get_port()))
            self._button_refresh_port_clicked_callback()
        if (not self.is_open) and current_is_open:
            self.log("Info::Connection established at {}.".format(self.umi_robot_shared_memory_receiver.get_port()))

        if current_is_open:
            self.is_open = True
            self.ui.label_connection_status.setText(
                "Connected to {}.".format(self.umi_robot_shared_memory_receiver.get_port()))
            try:
                q = self.umi_robot_shared_memory_receiver.get_q()
                self.ui.lineedit_q_0.setText(str(q[0]))
                self.ui.lineedit_q_1.setText(str(q[1]))
                self.ui.lineedit_q_2.setText(str(q[2]))
                self.ui.lineedit_q_3.setText(str(q[3]))
                self.ui.lineedit_q_4.setText(str(q[4]))
                self.ui.lineedit_q_5.setText(str(q[5]))

                potentiometer_values = self.umi_robot_shared_memory_receiver.get_potentiometer_values()

                if self._are_potentiometers_calibrated():
                    for i in range(0, 6):
                        self.potentiometer_qd[i] = int(
                            float(potentiometer_values[i] / (self.pot_max[i] - self.pot_min[i])) * 120.0) - 60

                self.ui.lineedit_pot_0.setText(str(self.potentiometer_qd[0]))
                self.ui.lineedit_pot_1.setText(str(self.potentiometer_qd[1]))
                self.ui.lineedit_pot_2.setText(str(self.potentiometer_qd[2]))
                self.ui.lineedit_pot_3.setText(str(self.potentiometer_qd[3]))
                self.ui.lineedit_pot_4.setText(str(self.potentiometer_qd[4]))
                self.ui.lineedit_pot_5.setText(str(self.potentiometer_qd[5]))

                if self.pot_min[0] is not None:
                    self.ui.lineedit_pot_min_0.setText("{0:.3g} V".format(self.pot_min[0]))
                if self.pot_min[1] is not None:
                    self.ui.lineedit_pot_min_1.setText("{0:.3g} V".format(self.pot_min[1]))
                if self.pot_min[2] is not None:
                    self.ui.lineedit_pot_min_2.setText("{0:.3g} V".format(self.pot_min[2]))
                if self.pot_min[3] is not None:
                    self.ui.lineedit_pot_min_3.setText("{0:.3g} V".format(self.pot_min[3]))
                if self.pot_min[4] is not None:
                    self.ui.lineedit_pot_min_4.setText("{0:.3g} V".format(self.pot_min[4]))
                if self.pot_min[5] is not None:
                    self.ui.lineedit_pot_min_5.setText("{0:.3g} V".format(self.pot_min[5]))

                if self.pot_max[0] is not None:
                    self.ui.lineedit_pot_max_0.setText("{0:.3g} V".format(self.pot_max[0]))
                if self.pot_max[1] is not None:
                    self.ui.lineedit_pot_max_1.setText("{0:.3g} V".format(self.pot_max[1]))
                if self.pot_max[2] is not None:
                    self.ui.lineedit_pot_max_2.setText("{0:.3g} V".format(self.pot_max[2]))
                if self.pot_max[3] is not None:
                    self.ui.lineedit_pot_max_3.setText("{0:.3g} V".format(self.pot_max[3]))
                if self.pot_max[4] is not None:
                    self.ui.lineedit_pot_max_4.setText("{0:.3g} V".format(self.pot_max[4]))
                if self.pot_max[5] is not None:
                    self.ui.lineedit_pot_max_5.setText("{0:.3g} V".format(self.pot_max[5]))

                if self.ui.checkbox_teleoperation.isChecked():
                    if self._are_potentiometers_calibrated():
                        for i in range(0, 6):
                            self.qd[i] = self.potentiometer_qd[i]

                if self.ui.checkbox_potentiometer_cal.isChecked():
                    for i in range(0, 6):
                        if self.pot_min[i] is None:
                            self.pot_min[i] = potentiometer_values[i]
                        if self.pot_max[i] is None:
                            self.pot_max[i] = potentiometer_values[i]

                    self.pot_min[0] = min(self.pot_min[0], potentiometer_values[0])
                    self.pot_min[1] = min(self.pot_min[1], potentiometer_values[1])
                    self.pot_min[2] = min(self.pot_min[2], potentiometer_values[2])
                    self.pot_min[3] = min(self.pot_min[3], potentiometer_values[3])
                    self.pot_min[4] = min(self.pot_min[4], potentiometer_values[4])
                    self.pot_min[5] = min(self.pot_min[5], potentiometer_values[5])

                    self.pot_max[0] = max(self.pot_max[0], potentiometer_values[0])
                    self.pot_max[1] = max(self.pot_max[1], potentiometer_values[1])
                    self.pot_max[2] = max(self.pot_max[2], potentiometer_values[2])
                    self.pot_max[3] = max(self.pot_max[3], potentiometer_values[3])
                    self.pot_max[4] = max(self.pot_max[4], potentiometer_values[4])
                    self.pot_max[5] = max(self.pot_max[5], potentiometer_values[5])

                    if abs(self.pot_min[0] - self.pot_max[0]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_0.setChecked(True)
                    if abs(self.pot_min[1] - self.pot_max[1]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_1.setChecked(True)
                    if abs(self.pot_min[2] - self.pot_max[2]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_2.setChecked(True)
                    if abs(self.pot_min[3] - self.pot_max[3]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_3.setChecked(True)
                    if abs(self.pot_min[4] - self.pot_max[4]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_4.setChecked(True)
                    if abs(self.pot_min[5] - self.pot_max[5]) > self.potentiometer_calibration_difference:
                        self.ui.radio_pot_5.setChecked(True)

                self.umi_robot_shared_memory_receiver.send_qd(self.qd)
            except Exception as e:
                self.log("Error::_timer_callback" + str(e))
        else:
            self.is_open = False
            self.ui.label_connection_status.setText("Disconnected.")

    def _are_potentiometers_calibrated(self):
        if self.ui.radio_pot_0.isChecked() and \
                self.ui.radio_pot_1.isChecked() and \
                self.ui.radio_pot_2.isChecked() and \
                self.ui.radio_pot_3.isChecked() and \
                self.ui.radio_pot_4.isChecked() and \
                self.ui.radio_pot_5.isChecked():
            return True
        else:
            return False

    def _button_connect_port_clicked_callback(self):
        current_selected_port = self.ui.combobox_port.currentText()
        if (current_selected_port == self.umi_robot_shared_memory_receiver.get_port()) and \
                self.umi_robot_shared_memory_receiver.is_open():
            self.log("Log::Already connected to {}.".format(current_selected_port))
            return
        if current_selected_port != "":
            self.umi_robot_shared_memory_receiver.send_port(current_selected_port)
            self.log("Info::Requesting connection to {}...".format(current_selected_port))
        else:
            self.log("Warning::No port selected.")

    def _button_refresh_port_clicked_callback(self):
        import serial.tools.list_ports
        self.ui.combobox_port.clear()
        ports_list = serial.tools.list_ports.comports()
        for port in ports_list:
            self.ui.combobox_port.addItem(str(port.name))
            if "Arduino Uno" in port.description:
                self.log("Info::Arduino Uno found at {}.".format(port.name))
        if len(ports_list) == 0:
            self.log("Warning::Updated ports but no ports available.")
        else:
            self.log("Info::Found {} serial ports.".format(len(ports_list)))

    @staticmethod
    def run(shared_memory_info, lock):
        umi_robot_shared_memory_receiver = UMIRobotSharedMemoryReceiver(shared_memory_info, lock)
        app = QApplication([])
        myapp = UMIRobotMainWindow(umi_robot_shared_memory_receiver)

        # Set app icon
        path = os.path.dirname(os.path.abspath(__file__))
        app.setWindowIcon(QIcon(os.path.join(path, 'icon.png')))

        # Set theme for Windows and fix icon not showing on taskbar
        if os.name == 'nt':
            myapp.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt5'))
            # https://stackoverflow.com/questions/1551605/how-to-set-applications-taskbar-icon-in-windows-7
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID('utokyo.umirobot')

        myapp.show()

        try:
            app.exec_()
        except Exception as e:
            print("umirobot_main_window::run::Error::" + str(e))
        except KeyboardInterrupt:
            print("umirobot_main_window::run::Info::Interrupted by user.")

        umi_robot_shared_memory_receiver.send_shutdown_flag(True)
