# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 16:28:38 2021

@author: u7
"""

import numpy as np
import serial  
from PIL import Image

COM_PORT = 'COM18'    #Modify to your own com_port
BAUD_RATES = 57600    
ser = serial.Serial(COM_PORT, BAUD_RATES)   

cnt =0
jump = 1
row = 60
col = 80
rowcnt=0
colcnt = 0
img = np.zeros([row,col], dtype=np.int8)


try:
    while jump:
        while ser.in_waiting:          
            data_raw = ser.read()  
            data = ord(data_raw)   
            img[rowcnt][colcnt] = data+128
            #print('Received raw_data：', data_raw)
            #print('Received data：', data)
            colcnt+=1
            if(colcnt==col):
                colcnt=0
                rowcnt+=1
            cnt+=1
            if(cnt==row*col):
                print('end')
                jump=0;
                break           
    
    imgshow = Image.fromarray(img)
    imgshow.save('pic.png')
    imgshow.show()

except KeyboardInterrupt:
    ser.close()    
    print('Keyboard Interrupt！')
