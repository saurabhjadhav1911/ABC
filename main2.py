#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\RL\RL_SPIDER
#https://github.com/saurabhjadhav1911/ABC.git
#C:\Users\Public\RL\ABC\ABC
import socket
import sys
import os
#import main_test as Child
sys.path.append(os.path.join(os.path.dirname(__file__),'..'))
import misc
def save_file(data):
    run_file=os.path.join(os.path.dirname(__file__),'main_test.txt')
    with open(run_file,"w") as f:
        f.write(data)
    try:
        reload(Child)
    except:
        import main_test as Child
        print("imported")

def get_script():
	host=misc.get_sock_ip()
	port =5000
	s=socket.socket()
	flag2=True
	script=""
	while flag2:
		s.connect((host,port))
		while True:
			msg=s.recv(1024)
			if "END" in str(msg):
				s.close()
				msg=msg.replace("END",'')
				break
			script+=msg
		
		print(msg)
		print(script)
		#save_file(script)
		flag2=False
	return script



def run_script(data):

	try:
		exec(data)#Child.Main()
	except:
		exc_traceback=traceback.format_exc()
		print("err")
		misc.sprint(str(exc_traceback))
	try:
		misc.sprint("END")
	except:
		pass

try:

	from multiprocessing import Pool,Process
	import traceback
	#Main()
	#save_file("hyghxgcv")
	run_script(get_script())

except Exception as e:
	exc_traceback=traceback.format_exc()
	print(exc_traceback)
	logname=__file__.replace('.py','')
	logname+='.log'
	print("error see file {}".format(logname))
	with open(logname,"w") as f:
			f.write(str(exc_traceback))


