"""
Copyright (C) 2020 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
from umirobot.shared_memory.umirobot_shared_memory_common import shared_memory_map


class UMIRobotSharedMemoryReceiver:
    def __init__(self, shared_memory_lists_tuple):
        self.connection_information_dict = shared_memory_map
        self.connection_information, self.shareable_q, self.shareable_qd, self.shareable_potentiometer_values = shared_memory_lists_tuple
        self.dofs = len(self.shareable_q)
        self.n_potentiometers = len(self.shareable_potentiometer_values)

    def send_qd(self, qd):
        if qd is not None:
            if len(qd) == self.dofs:
                for i in range(0, self.dofs):
                    self.shareable_qd[i] = qd[i]

    def get_q(self):
        return list(self.shareable_q)

    def get_potentiometer_values(self):
        return list(self.shareable_potentiometer_values)

    def is_open(self):
        return list(self.connection_information)[self.connection_information_dict['is_open']]

    def send_port(self, port):
        if not self.connection_information[self.connection_information_dict['port_connect_signal']]:
            self.connection_information[self.connection_information_dict['port']] = port
            self.connection_information[self.connection_information_dict['port_connect_signal']] = True
        else:
            print("UMIRobotSharedMemoryReceiver::send_port::Unable to send port.")

    def get_port(self):
        return list(self.connection_information)[self.connection_information_dict['port']]

    def send_shutdown_flag(self, flag):
        self.connection_information[self.connection_information_dict['shutdown_flag']] = flag
