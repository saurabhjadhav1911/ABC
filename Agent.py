import multiprocessing
import numpy as np
from misc import *
import serial
import time
import cv2
from collections import deque
import Env
import Cam
import sys
import os
import traceback


class Agent():
    """docstring for Env"""
    def __init__(self,config):
        self.config=config
        print('Agent created')
        self.maze_size=config['Env_config']['maze_size']
        self.maze=100*np.ones((self.maze_size[0],self.maze_size[1]))
        self.verlines=np.ones((self.maze_size[0],self.maze_size[1]-2))
        self.horlines=np.ones((self.maze_size[0]-2,self.maze_size[1]-1))
        self.que=deque(maxlen=100)

        self.dir={'N':0,'E':1,'S':2,'W':3}
        self.start=[5,0]
        self.start_orientation=0
        self.pos=self.start
        self.current_target=None
        self.prev_pos=self.start
        self.actions=[0,1,2,3]
        self.orientation=self.start_orientation
        self.neighbours=[[-1,0],[0,1],[1,0],[0,-1]]
        self.dir_lines=[[-1,-1],[-1,0],[0,-1],[-1,-1]]

        self.ver_nolines=[[0,0],[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],[0,8],[5,0],
                          [5,1],[5,2],[5,3],[5,4],[5,5],[5,6],[5,7],[5,8],[5,9],[1,2],
                          [1,3],[1,4],[1,5],[1,6],[1,7],[1,4],[2,4],[3,4],[4,4],[5,4]]
        self.hor_nolines=[[1,4],[2,4],[3,4],[4,4],[1,5],[2,5],[3,5],[4,5],[1,8],[1,9]]

        self.hor_size=self.horlines.shape
        self.ver_size=self.verlines.shape
        for i,j in self.hor_nolines:
            self.horlines[i,j]=-1
        for i,j in self.ver_nolines:
            self.verlines[i,j]=-1

        #########for trial#################
        self.positions={'RP':[[1,2]],'ND':[[3,2],[5,3],[4,4],[5,8],[4,9],[3,10]],'TF':[[3,6]],'AL':[[3,0]],
                        'AR':[[3,0]],'IP':[[3,1],[3,7],[4,7],[5,7]],'IB':[[4,3],[3,4]],'RB':[[4,3]],'BB':[[3,4]]}
        ###################################

        self.dict={'RP':105,'ND':-10,'TF':-15,'AL':-20,'AR':-25,'IP':130,'IB':135,'RB':140,'BB':145}
        


        for key in self.dict.keys():
            for pos in self.positions[key]:
                self.maze[pos[0],pos[1]]=self.dict[key]

        self.pixelpercell=100
        self.img = np.zeros([self.pixelpercell*(self.maze.shape[0]-1),self.pixelpercell*(self.maze.shape[1]-1),3], np.uint8)
        self.img=self.load_background()

        print(self.maze)

    def remove_lines(self):
        pass

    def load_background(self):
        return cv2.imread('back.jpg')

    def flodfill(self,target):
        self.maze[target[0],target[1]]=0
        self.que.append(target)
        while self.que:
            point=self.que.popleft()
            print("point {} popped".format(point))
            value=self.maze[point[0],point[1]]
            for i in range(4):
                if i%2==0:
                    
                    negh=[point[0]+self.dir_lines[i][0],point[1]+self.dir_lines[i][1]]
                    if (negh[0]>-1) and (negh[1]>-1) and (negh[0]<self.ver_size[0]) and (negh[1]<self.ver_size[1]):
                        print("verlines cordinates{}".format(negh))
                        if(self.verlines[negh[0],negh[1]]==1):
                            negh_point=[point[0]+self.neighbours[i][0],point[1]+self.neighbours[i][1]]
                            print("verline {} is present".format(negh))
                            if self.maze[negh_point[0],negh_point[1]]>(value+1):
                                self.maze[negh_point[0],negh_point[1]]=(value+1)
                                print("point {} given value {}".format(negh_point,value+1))
                                self.que.append(negh_point)

                else:
                    negh=[point[0]+self.dir_lines[i][0],point[1]+self.dir_lines[i][1]]
                    if (negh[0]>-1) and (negh[1]>-1) and (negh[0]<self.hor_size[0]) and (negh[1]<self.hor_size[1]):
                        print("verlines cordinates{}".format(negh))
                        if(self.horlines[negh[0],negh[1]]==1):
                            negh_point=[point[0]+self.neighbours[i][0],point[1]+self.neighbours[i][1]]
                            print("horline {} is present".format(negh))
                            if self.maze[negh_point[0],negh_point[1]]>(value+1):
                                self.maze[negh_point[0],negh_point[1]]=(value+1)
                                print("point {} given value {}".format(negh_point,value+1))
                                self.que.append(negh_point)


    def generate_background(self):
        self.img[:,:,:]=0
        for i in range(self.maze.shape[0]+1):
            a=int(self.pixelpercell*i)
            cv2.line(self.img,(0,a),(self.img.shape[1],a),[255,255,255],5)
        c=self.img.shape[1]
        for i in range(self.maze.shape[1]+1):
            a=int(self.pixelpercell*i)
            cv2.line(self.img,(a,0),(a,self.img.shape[0]),[255,255,255],5)

        for i,j in self.hor_nolines:
            a,b,c,d=self.pixelpercell*i,self.pixelpercell*(i+1),self.pixelpercell*j,self.pixelpercell*(j+1)
            cv2.line(self.img,(c+5,b),(d-5,b),[0,0,0],5)

        for i,j in self.ver_nolines:
            a,b,c,d=self.pixelpercell*i,self.pixelpercell*(i+1),self.pixelpercell*j,self.pixelpercell*(j+1)
            cv2.line(self.img,(d,a+5),(d,b-5),[0,0,0],5)

        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                c=self.pixelpercell*(i+0.05)
                d=self.pixelpercell*(j-0.05)
                
                a=c#+self.pixelpercell*0.225*self.neighbours[ac][0]
                b=d#+self.pixelpercell*0.225*self.neighbours[ac][1]
                cv2.putText( self.img,str(int(self.maze[i,j])),(int(b),int(a)),   cv2.FONT_HERSHEY_PLAIN, 1,(0, 0, 255), 2 )

        for i,row in enumerate(self.verlines):
            for j,val in enumerate(row):
                if [i,j] not in self.ver_nolines:
                    c=self.pixelpercell*(i+0.55)
                    d=self.pixelpercell*(j+0.90)
                    
                    a=c#+self.pixelpercell*0.225*self.neighbours[ac][0]
                    b=d#+self.pixelpercell*0.225*self.neighbours[ac][1]
                    cv2.putText( self.img,str(i)+','+str(j),(int(b),int(a)),   cv2.FONT_HERSHEY_PLAIN, 1,(255, 0, 0), 2 )

        for i,row in enumerate(self.horlines):
            for j,val in enumerate(row):
                if [i,j] not in self.hor_nolines:
                    c=self.pixelpercell*(i+0.95)
                    d=self.pixelpercell*(j+0.45)
                    
                    a=c#+self.pixelpercell*0.225*self.neighbours[ac][0]
                    b=d#+self.pixelpercell*0.225*self.neighbours[ac][1]
                    cv2.putText( self.img,str(i)+','+str(j),(int(b),int(a)),   cv2.FONT_HERSHEY_PLAIN, 1,(0, 255, 0), 2 )


        cv2.imshow('Environment',self.img)
        cv2.imwrite('back.jpg',self.img)
        cv2.waitKey(0)

        return self.img

    def render(self):

        self.env=self.img.copy()

        for i,row in enumerate(self.horlines):
            for j,v in enumerate(row):
                if v !=1:
                    a,b,c,d=self.pixelpercell*i,self.pixelpercell*(i+1),self.pixelpercell*j,self.pixelpercell*(j+1)
                    cv2.line(self.img,(c+5,b),(d-5,b),[0,0,0],5)

        for i,row in enumerate(self.verlines):
            for j,v in enumerate(row):
                if v != 1:
                    a,b,c,d=self.pixelpercell*i,self.pixelpercell*(i+1),self.pixelpercell*j,self.pixelpercell*(j+1)
                    cv2.line(self.img,(d,a+5),(d,b-5),[0,0,0],5)

        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                a=self.pixelpercell*(i+0.07)#+self.pixelpercell*0.225*self.neighbours[ac][0]
                b=self.pixelpercell*(j-0.05)#+self.pixelpercell*0.225*self.neighbours[ac][1]
                value=str(list(self.dict.keys())[list(self.dict.values()).index(self.maze[i,j])] if self.maze[i,j] < 0 else int(self.maze[i,j]))
                cv2.putText( self.img,value,(int(b),int(a)),   cv2.FONT_HERSHEY_PLAIN, 1.4,(255,0 , 155), 2 )

        cv2.imshow('Environment',self.img)
        cv2.waitKey(0)
        
    def draw_block(self):
        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                spl=False
                if [i,j]==self.goal:
                    color=[0,255,0]
                    spl=True
                elif [i,j]==self.nogoal:
                    color=[0,0,255]
                    spl=True
                if [i,j] in self.obstacles:
                    color=[255,0,0]
                    spl=True
                if spl:
                    a=self.pixelpercell*i
                    c=self.pixelpercell*(i+1)
                    b=self.pixelpercell*j
                    d=self.pixelpercell*(j+1)
                    cv2.rectangle(self.img,(b,a),(d,c),color, thickness=-1)
                    #print(a,b,c,d)

def Main():
    config=read_config()
    agent=Agent(config)
    #agent.flodfill(agent.positions['TF'][0])
    agent.flodfill(agent.start)
    agent.render()
    print(agent.maze)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    Main()
    try:

        pass

    except Exception as e:
        exc_traceback=traceback.format_exc()
        print(exc_traceback)
        logname=__file__.replace('.py','.log')
        print("error see file {}".format(logname))
        with open(logname,"w") as f:
                f.write(str(exc_traceback))