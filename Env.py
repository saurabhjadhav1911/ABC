#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\RL
#https://github.com/saurabhjadhav1911/RL.git
from misc import *
import serial
import multiprocessing
import time
import traceback

value=None
nf=0
class Env():
    """docstring for Env"""
    def __init__(self,config):
        print('Env created')
        self.config=config
        self.ser=self.get_Serial()
        self.data=""

    def get_Serial(self):
        return serial.Serial(self.config['Serial_config']['port'],baudrate=self.config['Serial_config']['baud'],timeout=self.config['Serial_config']['timeout'])

    def reset(self):
        self.action(self.default_action)
        return self.read_state()

    def action(self,act):
        line="G "+" ".join(map(str,act))
        print(line)
        #self.serial.write(line)

    def read_write_state(self,q,r):

        flag=False
        while r.empty() is False:
            flag=True
            arr=r.get()
            arr+="|"
            #unicode(s, "utf-8")
            #print(arr)
            self.ser.write(arr.encode())

        if(self.ser.inWaiting()>0):
            c=self.ser.read() 
            try:
                c=str(c,'utf-8')
            except Exception as e:
                print(e)
                c=""
            #print(c)
            c=str(c)
            #print(c)
            try:
                if c is '|':
                    q.put(self.data)
                    self.data=""
                else:
                	self.data+=c
            except Exception as e:
            	exc_traceback=traceback.format_exc()
            	print(exc_traceback)
            	#pass


    def run(self,q,r):
        #ser=serial.Serial(config['Serial_config']['port'],baudrate=config['Serial_config']['baud'],timeout=config['Serial_config']['timeout'])
        time.sleep(5)
        while True:
            self.read_write_state(q,r)

if __name__ == '__main__':
    
    env=Env(read_config())
    q=multiprocessing.Queue()
    #process=multiprocessing.Process(target= plot.add_elements,args=(q,plot.mem,plot.back_image, plot.size))
    process=multiprocessing.Process(target=env.run,args=(q,))
    process.start()
    process.join()

