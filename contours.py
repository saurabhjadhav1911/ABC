import numpy as np
import cv2
import time

left=cv2.imread('left_arrow.jpg',cv2.IMREAD_GRAYSCALE)
right=cv2.imread('right_arrow.jpg',cv2.IMREAD_GRAYSCALE)

thres=0.75

def main():
    last_time = time.time()
    v=cv2.VideoCapture(1)
    while(True):
        ret,frame=v.read()
        new_screen,arrow =cnt(frame)
        #print('Loop took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
        r,c,ch=new_screen.shape
        cv2.putText(new_screen, arrow, (int(c/2),int(r/2)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)
        cv2.imshow('window', new_screen)
        
        #cv2.imshow('window2', cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

def cnt(im):
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
                res_left,res_right=match_arrow(mat)
                #print(res_left,res_right)
                if res_left>mxl:
                    mxl=res_left
                if res_right>mxr:
                    mxr=res_right
            #print(mx)   
                
        #print(e)
    if max(mxl,mxr)>thres:
        if mxl>mxr:
            arrow="Left"
        else:
            arrow="Right"
    return im,arrow

def match_arrow(mat):
    res_left,res_right=0,0
    try:
        r=mat.shape[1]
        c=mat.shape[0]
        left_arr = cv2.resize(left,(r,c))
        right_arr = cv2.resize(right,(r,c))
        #print(left_arr.shape,right_arr.shape,mat.shape)
        res_left = cv2.matchTemplate(mat,left_arr,cv2.TM_CCOEFF_NORMED)
        res_right = cv2.matchTemplate(mat,right_arr,cv2.TM_CCOEFF_NORMED)
         
    except Exception as e:
        print(e)
    return res_left,res_right
    
main()
