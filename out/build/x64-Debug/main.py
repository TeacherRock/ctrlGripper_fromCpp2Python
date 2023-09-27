import ctypes
from ctypes import cdll, c_char_p, c_float, CDLL
import time
import os

dll = CDLL('ctrlGripper_dll.dll', winmode=0)
dll.setPos.argtypes = [ctypes.c_double]

if __name__ == "__main__":
    pos_request_ = 30.0
    pos_home     = 85.0
    # dll.iniGripper(7)

    dll.Active_on()
    os.system("pause")

    dll.setPos(pos_request_)
    dll.moveGripper()
    os.system("pause")

    dll.setPos(pos_home)
    dll.moveGripper()
    os.system("pause")

    dll.Active_off()
    os.system("pause")