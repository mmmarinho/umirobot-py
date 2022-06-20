# Form implementation generated from reading ui file 'umirobot_main_window.ui'
#
# Created by: PyQt6 UI code generator 6.3.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1197, 599)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icon.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.frame_3 = QtWidgets.QFrame(self.centralwidget)
        self.frame_3.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_3.setObjectName("frame_3")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame_3)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.frame_6 = QtWidgets.QFrame(self.frame_3)
        self.frame_6.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_6.setObjectName("frame_6")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame_6)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_manual_control_angle_change = QtWidgets.QLabel(self.frame_6)
        self.label_manual_control_angle_change.setObjectName("label_manual_control_angle_change")
        self.verticalLayout.addWidget(self.label_manual_control_angle_change)
        self.slider_manual_control = QtWidgets.QSlider(self.frame_6)
        self.slider_manual_control.setMinimum(0)
        self.slider_manual_control.setMaximum(50)
        self.slider_manual_control.setProperty("value", 5)
        self.slider_manual_control.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider_manual_control.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBothSides)
        self.slider_manual_control.setTickInterval(5)
        self.slider_manual_control.setObjectName("slider_manual_control")
        self.verticalLayout.addWidget(self.slider_manual_control)
        self.gridLayout_2.addWidget(self.frame_6, 4, 4, 1, 1)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.label_4 = QtWidgets.QLabel(self.frame_3)
        self.label_4.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout.addWidget(self.label_4)
        self.checkbox_jointcontrol = QtWidgets.QCheckBox(self.frame_3)
        self.checkbox_jointcontrol.setEnabled(True)
        self.checkbox_jointcontrol.setText("")
        self.checkbox_jointcontrol.setChecked(False)
        self.checkbox_jointcontrol.setObjectName("checkbox_jointcontrol")
        self.horizontalLayout.addWidget(self.checkbox_jointcontrol)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.gridLayout_2.addLayout(self.horizontalLayout, 1, 4, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem2)
        self.label_5 = QtWidgets.QLabel(self.frame_3)
        self.label_5.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_2.addWidget(self.label_5)
        self.checkbox_teleoperation = QtWidgets.QCheckBox(self.frame_3)
        self.checkbox_teleoperation.setText("")
        self.checkbox_teleoperation.setObjectName("checkbox_teleoperation")
        self.horizontalLayout_2.addWidget(self.checkbox_teleoperation)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem3)
        self.gridLayout_2.addLayout(self.horizontalLayout_2, 1, 6, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_2.addItem(spacerItem4, 2, 5, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_2.addItem(spacerItem5, 4, 8, 1, 1)
        spacerItem6 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_2.addItem(spacerItem6, 2, 3, 1, 1)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        spacerItem7 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem7)
        self.label_6 = QtWidgets.QLabel(self.frame_3)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_3.addWidget(self.label_6)
        self.checkbox_potentiometer_cal = QtWidgets.QCheckBox(self.frame_3)
        self.checkbox_potentiometer_cal.setText("")
        self.checkbox_potentiometer_cal.setObjectName("checkbox_potentiometer_cal")
        self.horizontalLayout_3.addWidget(self.checkbox_potentiometer_cal)
        spacerItem8 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem8)
        self.gridLayout_2.addLayout(self.horizontalLayout_3, 1, 8, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.frame_3)
        self.label_2.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 1, 2, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame_3)
        self.label_7.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 1, 0, 1, 1)
        self.frame_5 = QtWidgets.QFrame(self.frame_3)
        self.frame_5.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_5.setObjectName("frame_5")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame_5)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.button_increase_q_1 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_1.setObjectName("button_increase_q_1")
        self.gridLayout_3.addWidget(self.button_increase_q_1, 1, 1, 1, 1)
        self.button_decrease_q_4 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_4.setObjectName("button_decrease_q_4")
        self.gridLayout_3.addWidget(self.button_decrease_q_4, 4, 0, 1, 1)
        self.button_decrease_q_5 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_5.setObjectName("button_decrease_q_5")
        self.gridLayout_3.addWidget(self.button_decrease_q_5, 5, 0, 1, 1)
        self.button_decrease_q_1 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_1.setObjectName("button_decrease_q_1")
        self.gridLayout_3.addWidget(self.button_decrease_q_1, 1, 0, 1, 1)
        self.button_increase_q_5 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_5.setObjectName("button_increase_q_5")
        self.gridLayout_3.addWidget(self.button_increase_q_5, 5, 1, 1, 1)
        self.button_decrease_q_3 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_3.setObjectName("button_decrease_q_3")
        self.gridLayout_3.addWidget(self.button_decrease_q_3, 3, 0, 1, 1)
        self.button_decrease_q_0 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_0.setObjectName("button_decrease_q_0")
        self.gridLayout_3.addWidget(self.button_decrease_q_0, 0, 0, 1, 1)
        self.button_increase_q_0 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_0.setObjectName("button_increase_q_0")
        self.gridLayout_3.addWidget(self.button_increase_q_0, 0, 1, 1, 1)
        self.button_decrease_q_2 = QtWidgets.QPushButton(self.frame_5)
        self.button_decrease_q_2.setObjectName("button_decrease_q_2")
        self.gridLayout_3.addWidget(self.button_decrease_q_2, 2, 0, 1, 1)
        self.button_increase_q_2 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_2.setObjectName("button_increase_q_2")
        self.gridLayout_3.addWidget(self.button_increase_q_2, 2, 1, 1, 1)
        self.button_increase_q_3 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_3.setObjectName("button_increase_q_3")
        self.gridLayout_3.addWidget(self.button_increase_q_3, 3, 1, 1, 1)
        self.button_increase_q_4 = QtWidgets.QPushButton(self.frame_5)
        self.button_increase_q_4.setObjectName("button_increase_q_4")
        self.gridLayout_3.addWidget(self.button_increase_q_4, 4, 1, 1, 1)
        self.gridLayout_2.addWidget(self.frame_5, 2, 4, 2, 1)
        spacerItem9 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_2.addItem(spacerItem9, 3, 1, 1, 1)
        self.frame = QtWidgets.QFrame(self.frame_3)
        self.frame.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame.setObjectName("frame")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.frame)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.lineedit_q_2 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_2.setEnabled(False)
        self.lineedit_q_2.setObjectName("lineedit_q_2")
        self.gridLayout_6.addWidget(self.lineedit_q_2, 2, 1, 1, 1)
        self.lineedit_q_0 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_0.setEnabled(False)
        self.lineedit_q_0.setObjectName("lineedit_q_0")
        self.gridLayout_6.addWidget(self.lineedit_q_0, 0, 1, 1, 1)
        self.label_q_0 = QtWidgets.QLabel(self.frame)
        self.label_q_0.setObjectName("label_q_0")
        self.gridLayout_6.addWidget(self.label_q_0, 0, 0, 1, 1)
        self.label_q_1 = QtWidgets.QLabel(self.frame)
        self.label_q_1.setObjectName("label_q_1")
        self.gridLayout_6.addWidget(self.label_q_1, 1, 0, 1, 1)
        self.lineedit_q_4 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_4.setEnabled(False)
        self.lineedit_q_4.setObjectName("lineedit_q_4")
        self.gridLayout_6.addWidget(self.lineedit_q_4, 4, 1, 1, 1)
        self.lineedit_q_3 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_3.setEnabled(False)
        self.lineedit_q_3.setObjectName("lineedit_q_3")
        self.gridLayout_6.addWidget(self.lineedit_q_3, 3, 1, 1, 1)
        self.label_q_2 = QtWidgets.QLabel(self.frame)
        self.label_q_2.setObjectName("label_q_2")
        self.gridLayout_6.addWidget(self.label_q_2, 2, 0, 1, 1)
        self.label_q_3 = QtWidgets.QLabel(self.frame)
        self.label_q_3.setObjectName("label_q_3")
        self.gridLayout_6.addWidget(self.label_q_3, 3, 0, 1, 1)
        self.lineedit_q_1 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_1.setEnabled(False)
        self.lineedit_q_1.setObjectName("lineedit_q_1")
        self.gridLayout_6.addWidget(self.lineedit_q_1, 1, 1, 1, 1)
        self.label_q_4 = QtWidgets.QLabel(self.frame)
        self.label_q_4.setObjectName("label_q_4")
        self.gridLayout_6.addWidget(self.label_q_4, 4, 0, 1, 1)
        self.label_q_5 = QtWidgets.QLabel(self.frame)
        self.label_q_5.setObjectName("label_q_5")
        self.gridLayout_6.addWidget(self.label_q_5, 5, 0, 1, 1)
        self.lineedit_q_5 = QtWidgets.QLineEdit(self.frame)
        self.lineedit_q_5.setEnabled(False)
        self.lineedit_q_5.setObjectName("lineedit_q_5")
        self.gridLayout_6.addWidget(self.lineedit_q_5, 5, 1, 1, 1)
        self.gridLayout_2.addWidget(self.frame, 2, 2, 2, 1)
        self.frame_7 = QtWidgets.QFrame(self.frame_3)
        self.frame_7.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_7.setObjectName("frame_7")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.frame_7)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.lineedit_pot_min_1 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_1.setEnabled(False)
        self.lineedit_pot_min_1.setObjectName("lineedit_pot_min_1")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_1, 1, 0, 1, 1)
        self.lineedit_pot_max_3 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_3.setEnabled(False)
        self.lineedit_pot_max_3.setObjectName("lineedit_pot_max_3")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_3, 3, 2, 1, 1)
        self.lineedit_pot_max_1 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_1.setEnabled(False)
        self.lineedit_pot_max_1.setObjectName("lineedit_pot_max_1")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_1, 1, 2, 1, 1)
        self.lineedit_pot_min_2 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_2.setEnabled(False)
        self.lineedit_pot_min_2.setObjectName("lineedit_pot_min_2")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_2, 2, 0, 1, 1)
        self.lineedit_pot_min_4 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_4.setEnabled(False)
        self.lineedit_pot_min_4.setObjectName("lineedit_pot_min_4")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_4, 4, 0, 1, 1)
        self.lineedit_pot_min_0 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_0.setEnabled(False)
        self.lineedit_pot_min_0.setObjectName("lineedit_pot_min_0")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_0, 0, 0, 1, 1)
        self.lineedit_pot_min_3 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_3.setEnabled(False)
        self.lineedit_pot_min_3.setObjectName("lineedit_pot_min_3")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_3, 3, 0, 1, 1)
        self.lineedit_pot_max_4 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_4.setEnabled(False)
        self.lineedit_pot_max_4.setObjectName("lineedit_pot_max_4")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_4, 4, 2, 1, 1)
        self.lineedit_pot_max_0 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_0.setEnabled(False)
        self.lineedit_pot_max_0.setObjectName("lineedit_pot_max_0")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_0, 0, 2, 1, 1)
        self.radio_pot_0 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_0.setEnabled(False)
        self.radio_pot_0.setText("")
        self.radio_pot_0.setAutoExclusive(False)
        self.radio_pot_0.setObjectName("radio_pot_0")
        self.gridLayout_7.addWidget(self.radio_pot_0, 0, 3, 1, 1)
        self.lineedit_pot_max_5 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_5.setEnabled(False)
        self.lineedit_pot_max_5.setObjectName("lineedit_pot_max_5")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_5, 5, 2, 1, 1)
        self.lineedit_pot_min_5 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_min_5.setEnabled(False)
        self.lineedit_pot_min_5.setObjectName("lineedit_pot_min_5")
        self.gridLayout_7.addWidget(self.lineedit_pot_min_5, 5, 0, 1, 1)
        self.radio_pot_1 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_1.setEnabled(False)
        self.radio_pot_1.setText("")
        self.radio_pot_1.setAutoExclusive(False)
        self.radio_pot_1.setObjectName("radio_pot_1")
        self.gridLayout_7.addWidget(self.radio_pot_1, 1, 3, 1, 1)
        self.lineedit_pot_max_2 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_max_2.setEnabled(False)
        self.lineedit_pot_max_2.setObjectName("lineedit_pot_max_2")
        self.gridLayout_7.addWidget(self.lineedit_pot_max_2, 2, 2, 1, 1)
        self.radio_pot_2 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_2.setEnabled(False)
        self.radio_pot_2.setText("")
        self.radio_pot_2.setAutoExclusive(False)
        self.radio_pot_2.setObjectName("radio_pot_2")
        self.gridLayout_7.addWidget(self.radio_pot_2, 2, 3, 1, 1)
        self.radio_pot_3 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_3.setEnabled(False)
        self.radio_pot_3.setText("")
        self.radio_pot_3.setAutoExclusive(False)
        self.radio_pot_3.setObjectName("radio_pot_3")
        self.gridLayout_7.addWidget(self.radio_pot_3, 3, 3, 1, 1)
        self.radio_pot_4 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_4.setEnabled(False)
        self.radio_pot_4.setText("")
        self.radio_pot_4.setAutoExclusive(False)
        self.radio_pot_4.setObjectName("radio_pot_4")
        self.gridLayout_7.addWidget(self.radio_pot_4, 4, 3, 1, 1)
        self.radio_pot_5 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_pot_5.setEnabled(False)
        self.radio_pot_5.setText("")
        self.radio_pot_5.setAutoExclusive(False)
        self.radio_pot_5.setObjectName("radio_pot_5")
        self.gridLayout_7.addWidget(self.radio_pot_5, 5, 3, 1, 1)
        self.lineedit_pot_0 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_0.setEnabled(False)
        self.lineedit_pot_0.setObjectName("lineedit_pot_0")
        self.gridLayout_7.addWidget(self.lineedit_pot_0, 0, 1, 1, 1)
        self.lineedit_pot_1 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_1.setEnabled(False)
        self.lineedit_pot_1.setObjectName("lineedit_pot_1")
        self.gridLayout_7.addWidget(self.lineedit_pot_1, 1, 1, 1, 1)
        self.lineedit_pot_2 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_2.setEnabled(False)
        self.lineedit_pot_2.setObjectName("lineedit_pot_2")
        self.gridLayout_7.addWidget(self.lineedit_pot_2, 2, 1, 1, 1)
        self.lineedit_pot_3 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_3.setEnabled(False)
        self.lineedit_pot_3.setObjectName("lineedit_pot_3")
        self.gridLayout_7.addWidget(self.lineedit_pot_3, 3, 1, 1, 1)
        self.lineedit_pot_4 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_4.setEnabled(False)
        self.lineedit_pot_4.setObjectName("lineedit_pot_4")
        self.gridLayout_7.addWidget(self.lineedit_pot_4, 4, 1, 1, 1)
        self.lineedit_pot_5 = QtWidgets.QLineEdit(self.frame_7)
        self.lineedit_pot_5.setEnabled(False)
        self.lineedit_pot_5.setObjectName("lineedit_pot_5")
        self.gridLayout_7.addWidget(self.lineedit_pot_5, 5, 1, 1, 1)
        self.gridLayout_2.addWidget(self.frame_7, 2, 8, 2, 1)
        spacerItem10 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_2.addItem(spacerItem10, 4, 2, 1, 1)
        self.label_image = QtWidgets.QLabel(self.frame_3)
        self.label_image.setText("")
        self.label_image.setPixmap(QtGui.QPixmap("umirobot.png"))
        self.label_image.setObjectName("label_image")
        self.gridLayout_2.addWidget(self.label_image, 3, 0, 1, 1)
        self.frame_4 = QtWidgets.QFrame(self.frame_3)
        self.frame_4.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_4.setObjectName("frame_4")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.frame_4)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.label_pot_0 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_0.setObjectName("label_pot_0")
        self.gridLayout_5.addWidget(self.label_pot_0, 0, 0, 1, 2)
        self.lineedit_teleop_1 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_1.setEnabled(False)
        self.lineedit_teleop_1.setObjectName("lineedit_teleop_1")
        self.gridLayout_5.addWidget(self.lineedit_teleop_1, 1, 2, 1, 1)
        self.lineedit_teleop_5 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_5.setEnabled(False)
        self.lineedit_teleop_5.setObjectName("lineedit_teleop_5")
        self.gridLayout_5.addWidget(self.lineedit_teleop_5, 5, 2, 1, 1)
        self.label_pot_2 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_2.setObjectName("label_pot_2")
        self.gridLayout_5.addWidget(self.label_pot_2, 2, 0, 1, 1)
        self.label_pot_3 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_3.setObjectName("label_pot_3")
        self.gridLayout_5.addWidget(self.label_pot_3, 3, 0, 1, 1)
        self.label_pot_5 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_5.setObjectName("label_pot_5")
        self.gridLayout_5.addWidget(self.label_pot_5, 5, 0, 1, 1)
        self.lineedit_teleop_2 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_2.setEnabled(False)
        self.lineedit_teleop_2.setObjectName("lineedit_teleop_2")
        self.gridLayout_5.addWidget(self.lineedit_teleop_2, 2, 2, 1, 1)
        self.lineedit_teleop_3 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_3.setEnabled(False)
        self.lineedit_teleop_3.setObjectName("lineedit_teleop_3")
        self.gridLayout_5.addWidget(self.lineedit_teleop_3, 3, 2, 1, 1)
        self.lineedit_teleop_0 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_0.setEnabled(False)
        self.lineedit_teleop_0.setObjectName("lineedit_teleop_0")
        self.gridLayout_5.addWidget(self.lineedit_teleop_0, 0, 2, 1, 1)
        self.label_pot_1 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_1.setObjectName("label_pot_1")
        self.gridLayout_5.addWidget(self.label_pot_1, 1, 0, 1, 2)
        self.label_pot_4 = QtWidgets.QLabel(self.frame_4)
        self.label_pot_4.setObjectName("label_pot_4")
        self.gridLayout_5.addWidget(self.label_pot_4, 4, 0, 1, 1)
        self.lineedit_teleop_4 = QtWidgets.QLineEdit(self.frame_4)
        self.lineedit_teleop_4.setEnabled(False)
        self.lineedit_teleop_4.setObjectName("lineedit_teleop_4")
        self.gridLayout_5.addWidget(self.lineedit_teleop_4, 4, 2, 1, 1)
        self.gridLayout_2.addWidget(self.frame_4, 2, 6, 2, 1)
        spacerItem11 = QtWidgets.QSpacerItem(0, 0, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_2.addItem(spacerItem11, 2, 7, 1, 1)
        spacerItem12 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_2.addItem(spacerItem12, 4, 6, 1, 1)
        self.gridLayout_9.addWidget(self.frame_3, 0, 0, 1, 3)
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setObjectName("label_13")
        self.gridLayout_9.addWidget(self.label_13, 1, 0, 1, 1)
        self.frame_10 = QtWidgets.QFrame(self.centralwidget)
        self.frame_10.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_10.setObjectName("frame_10")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.frame_10)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        spacerItem13 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem13)
        self.checkbox_communication_arduino_all = QtWidgets.QCheckBox(self.frame_10)
        self.checkbox_communication_arduino_all.setChecked(False)
        self.checkbox_communication_arduino_all.setObjectName("checkbox_communication_arduino_all")
        self.horizontalLayout_6.addWidget(self.checkbox_communication_arduino_all)
        spacerItem14 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem14)
        self.checkbox_communication_arduino_master_only = QtWidgets.QCheckBox(self.frame_10)
        self.checkbox_communication_arduino_master_only.setObjectName("checkbox_communication_arduino_master_only")
        self.horizontalLayout_6.addWidget(self.checkbox_communication_arduino_master_only)
        spacerItem15 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem15)
        self.gridLayout_4.addLayout(self.horizontalLayout_6, 1, 0, 1, 1)
        self.frame_9 = QtWidgets.QFrame(self.frame_10)
        self.frame_9.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_9.setObjectName("frame_9")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.frame_9)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.label_11 = QtWidgets.QLabel(self.frame_9)
        self.label_11.setObjectName("label_11")
        self.gridLayout_8.addWidget(self.label_11, 0, 0, 1, 1)
        self.lineedit_vrep_ip = QtWidgets.QLineEdit(self.frame_9)
        self.lineedit_vrep_ip.setMinimumSize(QtCore.QSize(150, 0))
        self.lineedit_vrep_ip.setObjectName("lineedit_vrep_ip")
        self.gridLayout_8.addWidget(self.lineedit_vrep_ip, 0, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.frame_9)
        self.label_10.setObjectName("label_10")
        self.gridLayout_8.addWidget(self.label_10, 1, 0, 1, 1)
        self.lineedit_vrep_port = QtWidgets.QLineEdit(self.frame_9)
        self.lineedit_vrep_port.setObjectName("lineedit_vrep_port")
        self.gridLayout_8.addWidget(self.lineedit_vrep_port, 1, 1, 1, 1)
        self.label_connection_status_vrep = QtWidgets.QLabel(self.frame_9)
        self.label_connection_status_vrep.setObjectName("label_connection_status_vrep")
        self.gridLayout_8.addWidget(self.label_connection_status_vrep, 2, 0, 1, 2)
        self.button_connect_vrep = QtWidgets.QPushButton(self.frame_9)
        self.button_connect_vrep.setObjectName("button_connect_vrep")
        self.gridLayout_8.addWidget(self.button_connect_vrep, 3, 0, 1, 2)
        self.gridLayout_4.addWidget(self.frame_9, 2, 2, 1, 1)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        spacerItem16 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem16)
        self.label_9 = QtWidgets.QLabel(self.frame_10)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_5.addWidget(self.label_9)
        self.checkbox_communication_coppeliasim = QtWidgets.QCheckBox(self.frame_10)
        self.checkbox_communication_coppeliasim.setText("")
        self.checkbox_communication_coppeliasim.setObjectName("checkbox_communication_coppeliasim")
        self.horizontalLayout_5.addWidget(self.checkbox_communication_coppeliasim)
        spacerItem17 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem17)
        self.gridLayout_4.addLayout(self.horizontalLayout_5, 1, 2, 1, 1)
        spacerItem18 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_4.addItem(spacerItem18, 2, 1, 1, 1)
        self.frame_2 = QtWidgets.QFrame(self.frame_10)
        self.frame_2.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout.setObjectName("gridLayout")
        self.button_connect_port = QtWidgets.QPushButton(self.frame_2)
        self.button_connect_port.setObjectName("button_connect_port")
        self.gridLayout.addWidget(self.button_connect_port, 4, 0, 1, 2)
        self.combobox_port = QtWidgets.QComboBox(self.frame_2)
        self.combobox_port.setMinimumSize(QtCore.QSize(120, 0))
        self.combobox_port.setObjectName("combobox_port")
        self.gridLayout.addWidget(self.combobox_port, 1, 0, 1, 1)
        self.label_connection_status = QtWidgets.QLabel(self.frame_2)
        self.label_connection_status.setObjectName("label_connection_status")
        self.gridLayout.addWidget(self.label_connection_status, 3, 0, 1, 2)
        self.button_refresh_port = QtWidgets.QPushButton(self.frame_2)
        self.button_refresh_port.setObjectName("button_refresh_port")
        self.gridLayout.addWidget(self.button_refresh_port, 1, 1, 1, 1)
        spacerItem19 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout.addItem(spacerItem19, 2, 0, 1, 1)
        self.gridLayout_4.addWidget(self.frame_2, 2, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.frame_10)
        self.label_3.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 0, 0, 1, 1)
        self.gridLayout_9.addWidget(self.frame_10, 4, 0, 3, 2)
        spacerItem20 = QtWidgets.QSpacerItem(20, 22, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_9.addItem(spacerItem20, 7, 0, 1, 1)
        spacerItem21 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_9.addItem(spacerItem21, 7, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.gridLayout_9.addWidget(self.label, 1, 2, 1, 1)
        self.frame_8 = QtWidgets.QFrame(self.centralwidget)
        self.frame_8.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_8.setObjectName("frame_8")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_8)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.textedit_status = QtWidgets.QTextEdit(self.frame_8)
        self.textedit_status.setEnabled(False)
        self.textedit_status.setObjectName("textedit_status")
        self.verticalLayout_2.addWidget(self.textedit_status)
        self.gridLayout_9.addWidget(self.frame_8, 4, 2, 3, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1197, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "UMI Robot Control Console"))
        self.label_manual_control_angle_change.setText(_translate("MainWindow", "Angle change per click"))
        self.label_4.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Manual Control</span></p></body></html>"))
        self.label_5.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Teleoperation</span></p></body></html>"))
        self.label_6.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Potentiometer Cal</span></p></body></html>"))
        self.label_2.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Joint Values</span></p></body></html>"))
        self.label_7.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">UMI Robot</span></p></body></html>"))
        self.button_increase_q_1.setText(_translate("MainWindow", ">>"))
        self.button_decrease_q_4.setText(_translate("MainWindow", "<<"))
        self.button_decrease_q_5.setText(_translate("MainWindow", "<<"))
        self.button_decrease_q_1.setText(_translate("MainWindow", "<<"))
        self.button_increase_q_5.setText(_translate("MainWindow", ">>"))
        self.button_decrease_q_3.setText(_translate("MainWindow", "<<"))
        self.button_decrease_q_0.setText(_translate("MainWindow", "<<"))
        self.button_increase_q_0.setText(_translate("MainWindow", ">>"))
        self.button_decrease_q_2.setText(_translate("MainWindow", "<<"))
        self.button_increase_q_2.setText(_translate("MainWindow", ">>"))
        self.button_increase_q_3.setText(_translate("MainWindow", ">>"))
        self.button_increase_q_4.setText(_translate("MainWindow", ">>"))
        self.label_q_0.setText(_translate("MainWindow", "q0"))
        self.label_q_1.setText(_translate("MainWindow", "q1"))
        self.label_q_2.setText(_translate("MainWindow", "q2"))
        self.label_q_3.setText(_translate("MainWindow", "q3"))
        self.label_q_4.setText(_translate("MainWindow", "q4"))
        self.label_q_5.setText(_translate("MainWindow", "q5"))
        self.label_pot_0.setText(_translate("MainWindow", "qd0"))
        self.label_pot_2.setText(_translate("MainWindow", "qd2"))
        self.label_pot_3.setText(_translate("MainWindow", "qd3"))
        self.label_pot_5.setText(_translate("MainWindow", "qd5"))
        self.label_pot_1.setText(_translate("MainWindow", "qd1"))
        self.label_pot_4.setText(_translate("MainWindow", "qd4"))
        self.label_13.setText(_translate("MainWindow", "Communication"))
        self.checkbox_communication_arduino_all.setText(_translate("MainWindow", "All"))
        self.checkbox_communication_arduino_master_only.setText(_translate("MainWindow", "Master only"))
        self.label_11.setText(_translate("MainWindow", "IP"))
        self.lineedit_vrep_ip.setText(_translate("MainWindow", "127.0.0.1"))
        self.label_10.setText(_translate("MainWindow", "Port"))
        self.lineedit_vrep_port.setText(_translate("MainWindow", "20000"))
        self.label_connection_status_vrep.setText(_translate("MainWindow", "Disconnected."))
        self.button_connect_vrep.setText(_translate("MainWindow", "Connect"))
        self.label_9.setText(_translate("MainWindow", "CoppeliaSim"))
        self.button_connect_port.setText(_translate("MainWindow", "Connect"))
        self.label_connection_status.setText(_translate("MainWindow", "Disconnected."))
        self.button_refresh_port.setText(_translate("MainWindow", "Refresh"))
        self.label_3.setText(_translate("MainWindow", "Arduino"))
        self.label.setText(_translate("MainWindow", "Information"))
