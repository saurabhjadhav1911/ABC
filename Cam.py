#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\IRC\ABC
#C:\software\rl\ABC
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

minRadius=50
maxRadius=200
threshold = 0.8

class Cam():
    
    def __init__(self):
        self.minRadius=50
        self.maxRadius=200
        self.threshold = 0.8
        self.logo=cv2.imread('tflogo.png',cv2.IMREAD_GRAYSCALE)
        self.v=cv2.VideoCapture(0)
        self.ret,self.frame=self.v.read()
        self.cimg=self.frame.copy()
        self.render=True
        self.dict={}

    

    def logo_detect(self):
        logo_flag=False
        logo_frames=0
        for i in range(20):
            self.ret,self.frame=self.v.read()
            #img=pattern_rec(frame)
            #print(ret)
            self.img,self.val=self.logo_rec()
            if self.val>self.threshold:
                logo_frames+=1
            if (cv2.waitKey(10)==ord('q')):
                break
            if logo_frames>2:
                logo_flag=True
                break
        self.v.release()
        if self.render and logo_flag:
            self.cimg=self.frame.copy()
            cv2.circle(self.cimg, (self.dict['center_x'], self.dict['center_y']), self.dict['radius'], (0, 0, 255), 3, cv2.LINE_AA)
        
        return logo_flag


    def arrow_detect(self):
        arr_flag=False
        arr_frames=0
        for i in range(20):
            self.ret,self.frame=self.v.read()
            #img=pattern_rec(frame)
            #print(ret)
            self.img,self.val=self.logo_rec()
            if self.val>self.threshold:
                arr_frames+=1
            if (cv2.waitKey(10)==ord('q')):
                break
            if arr_frames>2:
                arr_flag=True
                break
        self.v.release()
        if self.render and arr_flag:
            self.cimg=self.frame.copy()
            cv2.circle(self.cimg, (self.dict['center_x'], self.dict['center_y']), self.dict['radius'], (0, 0, 255), 3, cv2.LINE_AA)
        
        return arr_flag

    def pattern_rec(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,100,255,0)
        
        im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img,contours,-1,(0,255,0),3)
        return img

    def logo_rec(self):
        #print(type(self.frame))
        gray_img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        img`=cv2.GaussianBlur(gray_img,(3,3),0)
         # numpy function

        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 10, np.array([]), 100, 30, minRadius, maxRadius)

        if circles is not None: # Check if circles have been found and only then iterate over these and add them to the image

            a, b, c = circles.shape
            #print(a,b,c)
            mx=0
            for n,[center_x,center_y,radius] in enumerate(circles[0]):
                gray_crop=gray_img[int(center_y-radius):int(center_y+radius),int(center_x-radius):int(center_x+radius)]
                #thres_crop = th = cv2.adaptiveThreshold(gray_crop, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
                try:
                    r=gray_crop.shape[1]
                    c=gray_crop.shape[0]
                    res_logo = cv2.resize(logo,(r,c))
                    #print(gray_crop.shape,res_logo.shape)
                    
                    res = cv2.matchTemplate(gray_crop,res_logo,cv2.TM_CCOEFF_NORMED)
                    if max(res)>mx:
                        mx=max(res)
                        self.dict['center_x']=center_x
                        self.dict['center_y']=center_y
                        self.dict['radius']=radius

                        #print(mx)                        
                except:
                    pass
        
        return self.cimg,mx

def find_logo():
    cam=Cam()
    if cam.logo_detect():
        print('tflogo detected')
        cv2.imshow('window',cam.cimg)
    else:
        print('tflogo not found')

def find_arrow():
    cam=Cam()
    direction,flag=cam.arrow_detect()
    if flag:
        print('arrow {}'.format())
        cv2.imshow('window',cam.cimg)
    else:
        print('tflogo not found')

#cv2.imshow('Original',frame)


#########################################################################################################
def detect(frame):
    blur=cv2.GaussianBlur(frame,(3,3),0)
    edges = cv2.Canny(blur,100,200,apertureSize = 3)
    #cv2.imshow('edges',edges)
    #cv2.waitKey(0)
    minLineLength = 50
    maxLineGap = 10
    try:
        lines = cv2.HoughLinesP(edges,1,np.pi/180,minLineLength,minLineLength,maxLineGap)

        for line in lines:
            #print(line)
            x1,y1,x2,y2=line[0]
            cv2.line(edges,(x1,y1),(x2,y2),(255,255,255),3)
    except Exception as e:
        print(e)
        
    return edges
 
def arrow_test():
    last_time = time.time()
    v=cv2.VideoCapture(1)
    while(True):
        ret,frame=v.read()
        new_screen = detect(frame)
        print('Loop took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
        cv2.imshow('window', new_screen)
        #cv2.imshow('window2', cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
if __name__=='__main__':
    arrow_test()
