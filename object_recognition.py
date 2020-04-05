"""
Created on Tue Mar 14 21:10:40 2020

@author: psubacz

Installing opencv on linux: https://stackoverflow.com/questions/37188623/ubuntu-how-to-install-opencv-for-python3
"""
import numpy as np
import cv2, os

class Object_Recognition(object):
    '''
    Object recognition class that creates an
    '''
    def __init__(self):
        '''
        Creates the neural network and initializes the wieghts
        '''
        #filepath to the caffemodel and prototxt
        #get the current path
        cp = os.path.abspath(os.getcwd())
        found_caffemodel = False
        for file in os.listdir(cp):
            if file.endswith(".caffemodel"):
                found_caffemodel = True

        if found_caffemodel:
            caffemodel_fp = 'MobileNetSSD_deploy.caffemodel'
            prototxt_fp = 'MobileNetSSD_deploy.prototxt.txt'
            print('Loading\n',caffemodel_fp,'\n',prototxt_fp)
            self.model = cv2.dnn.readNetFromCaffe(prototxt_fp,caffemodel_fp)
        else:
            print('Caffe model not found, exiting...')
            exit()

        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
	            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	            "sofa", "train", "tvmonitor"]

    def predict(self,frame):
        '''

        '''
        frame = imutils.resize(frame, width=400)
        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
            0.007843, (300, 300), 127.5)
        # pass the blob through the network and obtain the detections and
        # predictions
        net.setInput(blob)
        detections = net.forward()
        return detections