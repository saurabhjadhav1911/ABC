import numpy as np
import cv2
import time

thres=0.9

def main():
    last_time = time.time()
    frame=cv2.imread('arrow.jpg')
    new_screen =cnt(frame)
    #print('Loop took {} seconds'.format(time.time()-last_time))
    last_time = time.time()
    r,c,ch=new_screen.shape
    cv2.imshow('window', new_screen)
    
def save_arrow(img):
    cv2.waitKey(0)
    cv2.imwrite('right_arrow.jpg',img)
    cv2.destroyAllWindows()

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
                save_arrow(mat)
    return im

main()
