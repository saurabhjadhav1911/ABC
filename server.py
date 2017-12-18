#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\RL\RL_SPIDER
#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\IRC\ABC
#https://github.com/saurabhjadhav1911/ABC.git
#C:\Users\Public\RL\ABC\ABC
import socket
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__),'..'))
import misc
import math
def send_file(filename,c):
	with open(filename,"r") as f:
		data=f.read()
		#print(info)
	#c.send(filename)
	n=0
	l=len(data)
	eof=True
	while eof:
	    if l>1000:
	        q=1000
	        l-=1000
	    else:
	        q=l
	        eof=False
	    c.send(data[n:n+q])
def Listen():
	host=misc.get_ip_mac()
	port =5000
	try:
	    run_file=sys.argv[1]
	except:
	    run_file=os.path.join(os.path.dirname(__file__),'main_test2.py')
	print run_file
	s=socket.socket()
	s.bind((host,port))
	print("server started")
	flag=True
	start=True

	while flag:
		s.listen(2)
		c,addr=s.accept()
		data=c.recv(1024)
		if not data:
			pass
		else:
			print(">>{}".format(data))

		if "END" in data:
			flag=False
		
	c.close()
def Main():
	host=misc.get_sock_ip()
	port =5000
	try:
	    run_file=sys.argv[1]
	except:
	    run_file=os.path.join(os.path.dirname(__file__),'main_test2.py')
	print run_file
	s=socket.socket()
	s.bind((host,port))
	print("server started")
	flag=True
	start=True
	while flag:
		s.listen(2)
		c,addr=s.accept()
		if start:
			#send_file(run_file,c)
			print(run_file)
			#send_file(run_file,c)
			start=False
			c.send("END")

		while True:
			data=c.recv(1024)
			if not data:
				break
			if "END" in data:
				flag=False
			print(">>{}".format(data))

		c.close()
'''

try:
	from misc import *
	import main_test as Child
	from multiprocessing import Pool,Process
	Main()

except Exception as e:
	print(e)
	logname=__file__.replace('.py','')
	logname+='.log'
	print("error see file {}".format(logname))
	with open(logname,"w") as f:
			f.write(str(e))
'''
if __name__=='__main__':
	Listen()


