import os
import sys
dec=[]
def conv():
    with open('meshflare4.txt',"r") as f:
        for line in f.readlines():
            if "void" in line:
                line=line.replace('\n','')
                line=line.replace('/','')
                line=line.replace('{','')
                line+=';'
                print(line)
                dec.append(line)
                
            
conv()
