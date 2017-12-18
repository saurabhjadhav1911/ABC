#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\RL\RL_SPIDER
#https://github.com/saurabhjadhav1911/ABC.git
#C:\Users\Public\RL\ABC\ABC
import socket
import sys
import misc

def sprint(msg):
	host =misc.get_ip_mac()
	#host =misc.get_ip_mac()

	print("host:{}".format(host))
	port =5000
	s=socket.socket()
	s.connect((host,port))
	s.send(msg)
	s.close()


if __name__=='__main__':
	sprint('aksdf')

