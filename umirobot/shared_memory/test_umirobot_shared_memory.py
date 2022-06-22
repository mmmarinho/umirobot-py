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
import multiprocessing as mp
import multiprocessing.managers as mm

from umirobot.shared_memory.umirobot_shared_memory_provider import UMIRobotSharedMemoryProvider
from umirobot.shared_memory.umirobot_shared_memory_receiver import UMIRobotSharedMemoryReceiver


def test_shared_memory_receiver(shared_memory_lists_tuple, lock):
    shared_memory_receiver = UMIRobotSharedMemoryReceiver(shared_memory_lists_tuple, lock=lock)
    while True:
        shared_memory_receiver.send_qd([1, 1, 1, 1, 1, 1])
        if shared_memory_receiver.get_q() == [2, 2, 2, 2, 2, 2]:
            print("Shared memory receiver received q.")
            if shared_memory_receiver.get_potentiometer_values() == [10, 20, 30, 40, 50, 60]:
                print("Shared memory receiver received potetiometer_values")
                if shared_memory_receiver.get_digital_in_values() == [1, 0, 1, 0, 1]:
                    print("Shared memory receiver received digital_in_values.")
                    break


if __name__ == '__main__':
    try:
        with mm.SharedMemoryManager() as smm:

            lock = mp.Lock()
            shared_memory_provider = UMIRobotSharedMemoryProvider(smm, lock=lock)
            shared_memory_provider_process = mp.Process(
                target=test_shared_memory_receiver,
                args=(shared_memory_provider.get_shared_memory_receiver_initializer_args(), lock)
            )
            shared_memory_provider_process.start()

            while True:
                pass
                shared_memory_provider.send_q([2, 2, 2, 2, 2, 2])
                shared_memory_provider.send_potentiometer_values([10, 20, 30, 40, 50, 60])
                shared_memory_provider.send_digital_in_values([1, 0, 1, 0, 1])
                if shared_memory_provider.get_qd() == [1, 1, 1, 1, 1, 1]:
                    print("Shared memory provider received qd.")
                    break

            shared_memory_provider_process.join()
    except Exception as e:
        print("test_umirobot_shared_memory::Error::" + str(e))
