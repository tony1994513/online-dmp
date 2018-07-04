#!/usr/bin/env python

import sys
import numpy as np
import rospy

from multiprocessing import Queue
from traj_gen import TarjGen
from traj_exec import TarjExec
import time

# traget_changed_Flag = False
# def callback(msg):
#     tmp = msg[0]
#     threshold = 0.1
#     if np.linalg.norm((msg - tmp)) > threshold:
#         traget_changed_Flag = True

if __name__ == '__main__':
    com_queue = Queue()

    tg = TarjGen(com_queue)
    te = TarjExec(com_queue)

    te.start()
    time.sleep(2)
    tg.start()

    while not rospy.is_shutdown():
        pass

    tg.shutdown()
    te.shutdown()

