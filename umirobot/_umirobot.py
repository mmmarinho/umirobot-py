"""
Copyright (C) 2020-2022 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import serial
from datetime import datetime


class UMIRobot:
    def __init__(self, baudrate=115200, timeout=0.5, n_servos=6, n_potentiometers=6, n_digital_inputs=5):
        """
        Making an instance of a UMIRobot does not start the communication automatically. To connect,
        use set_port(). The recommended way of using this class is with the 'with' statement, an example as follows:
        >> with UMIRobot() as umi_robot:
        >>  umi_robot.set_port('COM3')
        >>  umi_robot.update()
        >>  print(umi_robot.get_q())
        :param baudrate: The baudrate for the serial communication. Needs to match the one set in the UMIRobot.
        :param timeout: A timeout for waiting for serial messages. The default value is recommended.
        :param n_servos: The number of servos of your UMIRobot.
        :param n_potentiometers: The number of potentiometers read over serial.
        :param n_digital_inputs: The number of digital inputs read over serial.
        """
        self.serial_ = serial.Serial()
        self.serial_.baudrate = baudrate
        self.serial_.timeout = timeout
        self.n_servos = n_servos
        self.n_potentiometers = n_potentiometers
        self.n_digital_inputs = n_digital_inputs

        self.message_log = []

        self.__clear__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        self.print_log_to_file()

    def __clear__(self):
        """
        (re)initializes (non-const) member variables.
        """
        self.port = None
        self.qd = []
        self.q = []
        self.potentiometer_values = []
        self.digital_input_values = []
        for i in range(0, self.n_servos):
            self.qd.append(None)
            self.q.append(None)
        for i in range(0, self.n_potentiometers):
            self.potentiometer_values.append(None)
        for i in range(0, self.n_digital_inputs):
            self.digital_input_values.append(None)

    def set_port(self, port):
        """
        Sets the serial port connection. Automatically (re-)opens the serial communication with the new port.
        :param port: The serial communication port, for instance 'COM3' in Windows.
        """
        self.log("Info::Connecting to {}.".format(port))
        if port is not None:
            if self.is_open():
                self.close()
            self.port = port
            self.serial_.setPort(port)
            self.open()

    def get_port(self):
        """
        Gets the serial port connection.
        :return: The serial communication port, for instance 'COM3' in Windows. None if not initialized yet.
        """
        return self.port

    def print_log_to_file(self):
        """
        Print the messages logged with log() to a file. Automatically called on __exit__().
        """
        date_time_str = datetime.now().strftime("UMIRobot_%m_%d_%Y_%H_%M_%S.txt")
        with open(date_time_str, 'w') as f:
            for message in self.message_log:
                f.write("%s\n" % message)

    def log(self, msg):
        """
        Logs an error message.
        :param msg: The message to be sent.
        """
        date_time_str = datetime.now().strftime("%m/%d/%Y, %H:%M:%S ")
        msg = date_time_str + " UMIRobot::" + msg
        self.message_log.append(msg)

    def is_open(self):
        """
        Checks if the serial port connection is open.
        :return: True if the serial conncation is open, False otherwise.
        """
        return self.serial_.is_open

    def open(self):
        """
        Opens the serial port communication.
        """
        try:
            if self.serial_.is_open:
                self.log("Warning::open::Port was not closed.")
            else:
                self.serial_.open()
        except Exception as e:
            self.log("Error::open::" + str(e))

    def close(self):
        """
        Closes the serial port communication.
        """
        self.log("Info::Closing connection at {}.".format(str(self.port)))
        self.serial_.close()
        self.__clear__()

    def get_q(self):
        """
        Returns q.
        :return: a list with len(q)=dofs whose values can be None if uninitialized.
        """
        return self.q

    def get_potentiometer_values(self):
        """
        Returns the potentiometer values.
        :return: a list with len()=n_potentiometers whose values can be None if uninitialized
        """
        return self.potentiometer_values

    def get_digital_in_values(self):
        """
        Returns the digital input values.
        :return: a list with len()=n_digital_inputs whose values can be None if uninitialized
        """
        return self.digital_input_values

    def set_qd(self, qd):
        """
        Sets qd to be sent by the UMIRobot. Silently fails if qd is None, qd[i] is None, or if len(qd)!=dofs.
        :param qd: a list representing the desired joint values with len(qd)=dofs.
        """
        if qd is None:
            self.log("Warning::set_qd::Ignoring None qd.")
            return
        for i in range(0, len(qd)):
            if qd[i] is None:
                return
        if len(qd) is not self.n_servos:
            self.log("Warning::set_qd::Ignoring qd with len={} != self.dofs={}.".format(
                len(qd),
                self.n_servos
            ))
            return
        self.qd = qd

    def update(self):
        """
        If the serial connection is open, decodes the incoming messages with joint values
        and potentiometer values. It also sends encoded messages with the desired joint values.
        Communication errors are handled here.
        """
        if self.is_open():
            try:
                serial_bytes = self.serial_.readline()
                # Decodes input line
                serial_string = serial_bytes.decode("ascii")
                splitted_string = serial_string.split(" ")
                splitted_string_size = len(splitted_string)
                if splitted_string_size == self.n_servos + self.n_potentiometers + self.n_digital_inputs + 4:
                    if splitted_string[splitted_string_size-1] == "UMI\r\n":
                        i = 0
                        # Servos
                        n_servos_from_arduino = int(splitted_string[i])
                        if self.n_servos != n_servos_from_arduino:
                            raise RuntimeError("Servo count from arduino differs from expected count {}!={}.".format(
                                n_servos_from_arduino,
                                self.n_servos
                            ))
                        i = i + 1
                        for j in range(i, i + self.n_servos):
                            self.q[j-i] = int(splitted_string[j])
                        i = i + self.n_servos
                        # Potentiometers
                        n_potentiometers_from_arduino = int(splitted_string[i])
                        if self.n_potentiometers != n_potentiometers_from_arduino:
                            raise RuntimeError("Potentiometers count from arduino differs from expected count {}!={}.".format(
                                n_potentiometers_from_arduino,
                                self.n_potentiometers
                            ))
                        i = i + 1
                        for j in range(i, i + self.n_potentiometers):
                            self.potentiometer_values[j-i] = float(splitted_string[j]) * (5.0 / 1023.0)
                        i = i + self.n_potentiometers
                        # Digital Inputs
                        n_digital_inputs_from_arduino = int(splitted_string[i])
                        if self.n_digital_inputs != n_digital_inputs_from_arduino:
                            raise RuntimeError("Digital input count from arduino differs from expected count {}!={}.".format(
                                n_digital_inputs_from_arduino,
                                self.n_digital_inputs
                            ))
                        i = i + 1
                        for j in range(i, i + self.n_digital_inputs):
                            self.digital_input_values[j-i] = int(splitted_string[j])

                    # Encodes output line
                    if self.qd is not None:
                        command_string = b''
                        for i in range(0, self.n_servos):
                            command_string += bytes(str(self.qd[i]), "ascii")
                            if i < self.n_servos - 1:
                                command_string += b' '
                        command_string += b'\n'
                        self.serial_.write(command_string)
                        self.serial_.flush()

            except serial.SerialException as e:
                self.log("Error::update::" + str(e))
                self.log("Info::update::SerialException received, closing connection.")
                self.close()
            except Exception as e:
                self.log("Error::update::" + str(e))


if __name__ == '__main__':
    """An example of how to use this class to connect to a UMIRobot at 'COM3'"""
    with UMIRobot() as umirobot:
        umirobot.set_port('COM3')
        print("Move the robot to 10 degrees in each joint.")
        print("Press CTRL+C to end.")
        while True:
            try:
                umirobot.set_qd([10, 10, 10, 10, 10, 10])
                umirobot.update()
            except Exception as e:
                print("umirobot_test::Error::" + str(e))
            except KeyboardInterrupt:
                print("umirobot_test::Info::Execution ended by user.")
                print("Move the robot back to 0 degrees in each joint.")
                umirobot.set_qd([0, 0, 0, 0, 0, 0])
                umirobot.update()
                break

