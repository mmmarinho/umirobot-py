import multiprocessing
from umirobot import umirobot_communication_loop
from umirobot.gui import UMIRobotMainWindow

if __name__ == '__main__':
  multiprocessing.freeze_support()
  umirobot_communication_loop(UMIRobotMainWindow.run) 
