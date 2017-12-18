#C:\Users\saurabhj\OneDrive\Documents\Python Scripts\IRC\ABC
#C:\software\rl\ABC
import cv2
import numpy as np
import time

minRadius=50
maxRadius=200
threshold = 0.8
logo=cv2.imread('tflogo.png',cv2.IMREAD_GRAYSCALE)

def pattern_rec(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,100,255,0)
    
    im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours,-1,(0,255,0),3)
    return img

def logo_rec(src):
    gray_img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(gray_img, 5)
    cimg = src.copy() # numpy function

    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 10, np.array([]), 100, 30, minRadius, maxRadius)

    if circles is not None: # Check if circles have been found and only then iterate over these and add them to the image

        a, b, c = circles.shape
        mx=0
        for center_x,center_y,radius in circles[0]:
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
                    #print(mx)
                    
                    cv2.circle(cimg, (center_x, center_y), radius, (0, 0, 255), 3, cv2.LINE_AA)
            except:
                pass
            

    return cimg,mx

def Main():
    global img
    v=cv2.VideoCapture(0)
    print("cam init")
    print("cam start")
    flag=False
    number=0
    for i in range(20):
        ret,frame=v.read()
        #img=pattern_rec(frame)
        #print(ret)
        img,val=logo_rec(frame)
        if val>threshold:
            number+=1
        if (cv2.waitKey(10)==ord('q')):
            break
        if number>2:
            flag=True
            break
    v.release()
    
    return flag
if __name__=='__main__':
    if Main():
        global img
        print('tflogo detected')
        cv2.imshow('window',img)
    else:
        print('tflogo not found')

    cv2.waitKey(1000)
    cv2.destroyAllWindows()
