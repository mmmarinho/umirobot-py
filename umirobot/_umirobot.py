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
import serial
from datetime import datetime


class UMIRobot:
    def __init__(self, baudrate=115200, timeout=0.5, dofs=6, n_potentiometers=6):
        """
        Making an instance of a UMIRobot does not start the communication automatically. To connect,
        use set_port(). The recommended way of using this class is with the 'with' statement, an example as follows:
        >> with UMIRobot() as umi_robot:
        >>  umi_robot.set_port('COM3')
        >>  umi_robot.update()
        >>  print(umi_robot.get_q())
        :param baudrate: The baudrate for the serial communication. Needs to match the one set in the UMIRobot.
        :param timeout: A timeout for waiting for serial messages. The default value is recommended.
        :param dofs: The number of DoFs of your UMIRobot.
        """
        self.serial_ = serial.Serial()
        self.serial_.baudrate = baudrate
        self.serial_.timeout = timeout
        self.dofs = dofs
        self.n_potentiometers = n_potentiometers
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
        for i in range(0, self.dofs):
            self.qd.append(None)
            self.q.append(None)
        for i in range(0, self.n_potentiometers):
            self.potentiometer_values.append(None)

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
        :return: a list with len()=dofs whose values can be None if uninitialized
        """
        return self.potentiometer_values

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
        if len(qd) is not self.dofs:
            self.log("Warning::set_qd::Ignoring qd with len={} != self.dofs={}.".format(
                len(qd),
                self.dofs
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
                if len(splitted_string) == self.dofs + self.n_potentiometers + 1:
                    if splitted_string[self.dofs + self.n_potentiometers] == "UMI\r\n":
                        for i in range(0, self.dofs):
                            self.q[i] = int(splitted_string[i])
                        for i in range(0, self.n_potentiometers):
                            self.potentiometer_values[i] = float(splitted_string[i+self.dofs])*(5.0/1023.0)

                    # Encodes output line
                    if self.qd is not None:
                        command_string = b''
                        for i in range(0, self.dofs):
                            command_string += bytes(str(self.qd[i]), "ascii")
                            if i < self.dofs - 1:
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

