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
    
    def __init__(self,config):
        self.minRadius=50
        self.maxRadius=200
        self.threshold = 0.8
        self.arr_thres=0.75
        self.logo=cv2.imread('tflogo.png',cv2.IMREAD_GRAYSCALE)
        self.left=cv2.imread('left_arrow.jpg',cv2.IMREAD_GRAYSCALE)
        self.right=cv2.imread('right_arrow.jpg',cv2.IMREAD_GRAYSCALE)

        self.v=cv2.VideoCapture(1)
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


    def cnt(self,im):
        imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mxl=0
        mxr=0
        arrow="NotFound"
        for cnt in contours:
            e=0.05*cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt,e,True)
            ln=len(approx)
            if ln>3 and ln<8:
                #cv2.drawContours(im, [approx], -1, (0,255,0), 3)
                x,y,w,h = cv2.boundingRect(cnt)
                
                mat=imgray[y:y+h,x:x+w]
                if ln==5 and mat.shape[0]>20 and mat.shape[1]>20:
                    cv2.rectangle(im,(x,y),(x+w,y+h),(0,0,255),2)
                    res_left,res_right=self.match_arrow(mat)
                    #print(res_left,res_right)
                    if res_left>mxl:
                        mxl=res_left
                    if res_right>mxr:
                        mxr=res_right
                #print(mx)   
                    
            #print(e)
        if max(mxl,mxr)>self.arr_thres:
            if mxl>mxr:
                arrow="Left"
            else:
                arrow="Right"
        return im,arrow

    def match_arrow(self,mat):
        res_left,res_right=0,0
        try:
            r=mat.shape[1]
            c=mat.shape[0]
            left_arr = cv2.resize(self.left,(r,c))
            right_arr = cv2.resize(self.right,(r,c))
            #print(left_arr.shape,right_arr.shape,mat.shape)
            res_left = cv2.matchTemplate(mat,left_arr,cv2.TM_CCOEFF_NORMED)
            res_right = cv2.matchTemplate(mat,right_arr,cv2.TM_CCOEFF_NORMED)
             
        except Exception as e:
            print(e)
        return res_left,res_right
        

    def arrow_detect(self):
        left_frames=0
        right_frames=0
        arrow="NotFound"
        for i in range(20):
            self.ret,self.frame=self.v.read()
            #img=pattern_rec(frame)
            #print(ret)
            self.img,self.arrow=self.cnt(self.frame)
            if self.arrow=="Left":
                left_frames+=1
            if self.arrow=="Right":
                right_frames+=1
            if max(left_frames,right_frames)>2:
                if left_frames>right_frames:
                    arrow="Left"
                else:
                    arrow="Right"
                break
        self.v.release()
        if self.render and arrow!="NotFound":
            self.cimg=self.img
        return arrow


    def logo_rec(self):
        #print(type(self.frame))
        gray_img = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        img=cv2.GaussianBlur(gray_img,(3,3),0)
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
    arrow=cam.arrow_detect()
    print('arrow {}'.format(arrow))
    cv2.imshow('window',cam.cimg)

#cv2.imshow('Original',frame)


#########################################################################################################


if __name__=='__main__':
    find_arrow()
