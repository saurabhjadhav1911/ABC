import numpy as np
import cv2
import time

def main():
    last_time = time.time()
    v=cv2.VideoCapture(1)
    while(True):
        ret,frame=v.read()
        new_screen =cnt(frame)
        #print('Loop took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
        cv2.imshow('window', new_screen)
        
        #cv2.imshow('window2', cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

def cnt(im):
    imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        e=0.05*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt,e,True)
        ln=len(approx)
        if ln>3 and ln<8:
            cv2.drawContours(im, [approx], -1, (0,255,0), 3)
        #print(e)
    return im

main()
