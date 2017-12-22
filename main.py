#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\RL\RL_SPIDER
#https://github.com/saurabhjadhav1911/RL.git
#C:\Users\Public\RL\ABC\ABC
#C:\Users\vaibhav\Documents\Python\RL\RL_SPIDER

import multiprocessing
import numpy as np
from misc import *
import serial
import time
import cv2
from collections import deque
import Env
import Cam
import Agent
import sys
import os
import traceback
#sys.path.append(os.path.join(os.path.dirname(__file__),'..'))

def Cam_process_target(recieve_que,send_que,config):
    cam=Cam.Cam(config)
    cam.run(recieve_que,send_que)

def Env_process_target(recieve_que,send_que,config):
    print('Env process start')
    env=Env.Env(config)
    env.run(recieve_que,send_que)

def Agent_process_target(recieve_que,send_que,config,mode):
    print('Agent process start')
    agent=Agent.Agent(config,mode)
    agent.run(recieve_que,send_que)


def Main():
    multiprocessing.freeze_support()
    config=read_config()

    #initialise communicatoions between processes
    send_que=multiprocessing.Queue()
    recieve_que=multiprocessing.Queue()

    #process initialisation
    env_process=multiprocessing.Process(target=Env_process_target,args=(recieve_que,send_que,config))
    agent_process=multiprocessing.Process(target=Agent_process_target,args=(recieve_que,send_que,config,mode))
    #cam_process=multiprocessing.Process(target=Cam_process_target,args=(recieve_que,send_que,config))

    env_process.start()
    agent_process.start()
    #cam_process.start()
    
    agent_process.join()
    env_process.join()
    #cam_process.start()
    


try:

    Main()

except Exception as e:
    exc_traceback=traceback.format_exc()
    print(exc_traceback)
    logname=__file__.replace('.py','.log')
    print("error see file {}".format(logname))
    with open(logname,"w") as f:
            f.write(str(exc_traceback))


