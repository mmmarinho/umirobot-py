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


class UMIRobotSharedMemoryProvider:
    def __init__(self, shared_memory_manager, lock, dofs=6, n_potentiometers=6):
        self.shared_memory_manager = shared_memory_manager
        self.lock = lock
        self.dofs = dofs
        self.n_potentiometers = n_potentiometers
        self.connection_information_dict = shared_memory_map
        self.connection_information = shared_memory_manager.ShareableList(
            [dofs,
             n_potentiometers,
             False,
             "                                               ",
             False,
             False]
        )
        self.shareable_q = shared_memory_manager.ShareableList(
            [None]*dofs
        )
        self.shareable_qd = shared_memory_manager.ShareableList(
            [None]*dofs
        )
        self.shareable_potentiometer_values = shared_memory_manager.ShareableList(
            [None]*n_potentiometers
        )

    def get_shared_memory_receiver_initializer_args(self):
        return (self.connection_information,
                self.shareable_q,
                self.shareable_qd,
                self.shareable_potentiometer_values)

    def send_q(self, q):
        if q is not None:
            if len(q) == self.dofs:
                self.lock.acquire()
                for i in range(0, self.dofs):
                    self.shareable_q[i] = q[i]
                self.lock.release()

    def send_potentiometer_values(self, potentiometer_values):
        if potentiometer_values is not None:
            if len(potentiometer_values) == self.n_potentiometers:
                self.lock.acquire()
                for i in range(0, self.n_potentiometers):
                    self.shareable_potentiometer_values[i] = potentiometer_values[i]
                self.lock.release()

    def get_qd(self):
        self.lock.acquire()
        qd = list(self.shareable_qd)
        self.lock.release()
        return qd

    def send_is_open(self, is_open):
        self.lock.acquire()
        self.connection_information[self.connection_information_dict['is_open']] = is_open
        self.lock.release()

    def get_port(self):
        self.lock.acquire()
        port = self.connection_information[self.connection_information_dict['port']]
        self.lock.release()
        return port

    def get_port_connect_signal(self):
        self.lock.acquire()
        port_connect_signal = self.connection_information[self.connection_information_dict['port_connect_signal']]
        self.lock.release()
        return port_connect_signal

    def send_port_connect_signal(self, port_connect_signal):
        self.lock.acquire()
        self.connection_information[self.connection_information_dict['port_connect_signal']] = port_connect_signal
        self.lock.release()

    def get_shutdown_flag(self):
        self.lock.acquire()
        shutdown_flag = self.connection_information[self.connection_information_dict['shutdown_flag']]
        self.lock.release()
        return shutdown_flag

