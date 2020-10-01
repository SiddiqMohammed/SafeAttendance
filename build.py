
import cv2
import numpy as np
import face_recognition
import os
from datetime import datetime
import serial
import time
import json
# from PIL import ImageGrab
 
# arduinoData = serial.Serial('COM10', 9600, timeout=.1)
tempList = []
mList = []
tCheck = []
tempCounts = 0
textState = 0
start = 0
finish = 0

path = 'ImagesAttendance'
images = []
classNames = []
nameRepeater = []
nameList2 = []
nameCheck = []
timeIn = {}


with open('names.json') as f:
    timeOut = json.load(f)
print(timeOut)

myList = os.listdir(path)
print(myList)
for cl in myList:
    curImg = cv2.imread(f'{path}/{cl}')
    images.append(curImg)
    classNames.append(os.path.splitext(cl)[0])
print(classNames)
 
def findEncodings(images):
    encodeList = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encode = face_recognition.face_encodings(img)[0]
        encodeList.append(encode)
    return encodeList


def checkTemp():
    global tempCounts
    global textState
    global start
    while True:
        # cv2.putText(img, "Check you temp", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
        
        val = arduinoData.readline().decode().strip('\r\n')
        # print(val)
        
        try:
            if val != None:
                val = int(float(val))
                # print(type(val))
                print(val)
                if val < 36 and val > 30:
                    # print("val: ", val)
                    tempList.append(val)

                    if len(tempList) == 5:
                        numName = tempList.count(val)
                        if numName == 5:
                            tempList.clear()
                            markAttendance(name)
                            return
                        else:
                            tempList.clear()
                            tempCounts = tempCounts + 1
                            if tempCounts > 5:
                                tempCounts = 0
                                return
                else:
                    
                    tempCounts = tempCounts + 1
                    if tempCounts > 5:
                        tempCounts = 0

                        # tempTooHigh()
                        textState = 2
                        

                        return

        except:
            a = 0

 
def markAttendance(name):
    # tempTextShow()
    global textState
    textState = 1

    start = time.perf_counter()

    with open('Attendance.csv','r+') as f:
        myDataList = f.readlines()
        nameList = []
        now = datetime.now()
        actnow = int(now.strftime('%M'))

        for line in myDataList:
            entry = line.split(',')
            nameList.append(entry[0])

        if name not in timeIn and actnow - timeOut[name] > 1:
            now = datetime.now()
            dtString = now.strftime('%H:%M:%S')
            f.writelines(f'\n{name},{dtString}, IN')
            timeIn[name] = int(now.strftime('%M'))
            # print(timeIn)

        if name in timeIn and actnow - timeIn[name] > 1:
            now = datetime.now()
            dtString = now.strftime('%H:%M:%S')
            f.writelines(f'\n{name},{dtString}, OUT')           
            timeOut[name] = int(now.strftime('%M'))
            # print(timeOut)
            timeIn.pop(name)

def nameShow(text):
    global textState
    global start
    
    if textState == 0:
        cv2.putText(img, "Hello, " + text, (10,50), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
        cv2.putText(img, "Check you temp", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
    elif textState == 1:
        cv2.putText(img, "DONE", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
        # time.sleep(5)
        # textState = 0

        final = time.perf_counter()

        if final - start > 2:
            textState = 0


    elif textState == 2:    
        cv2.putText(img, "Your Temperature Is Too High!", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
        # time.sleep(5)
        # textState = 0

    
def newTemp():
    E = arduinoData.readline().decode().strip('\r\n')
    return E

# def tempTooHigh():
    
 
#### FOR CAPTURING SCREEN RATHER THAN WEBCAM
# def captureScreen(bbox=(300,300,690+300,530+300)):
#     capScr = np.array(ImageGrab.grab(bbox))
#     capScr = cv2.cvtColor(capScr, cv2.COLOR_RGB2BGR)
#     return capScr
 
encodeListKnown = findEncodings(images)
print('Encoding Complete')
 
cap = cv2.VideoCapture(0)

if cap.isOpened():
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float

arduinoData = serial.Serial('COM10', 9600, timeout=.1)

while True:
    # global tempCounts
    # global textState
    success, img = cap.read()
    #img = captureScreen()
    imgS = cv2.resize(img,(0,0),None,0.25,0.25)
    imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)

    # E = arduinoData.readline().decode().strip('\r\n')
 
    facesCurFrame = face_recognition.face_locations(imgS)
    encodesCurFrame = face_recognition.face_encodings(imgS,facesCurFrame)

    # arduinoData = serial.Serial('COM10', 9600, timeout=.1)
    

    # print(E)
    # print(type(E))

    for encodeFace,faceLoc in zip(encodesCurFrame,facesCurFrame):
        matches = face_recognition.compare_faces(encodeListKnown,encodeFace)
        faceDis = face_recognition.face_distance(encodeListKnown,encodeFace)
        #print(faceDis)
        matchIndex = np.argmin(faceDis)
 
        if matches[matchIndex]:
            name = classNames[matchIndex].upper()
            y1,x2,y2,x1 = faceLoc
            y1, x2, y2, x1 = y1*4,x2*4,y2*4,x1*4
            cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.rectangle(img,(x1,y2-35),(x2,y2),(0,255,0),cv2.FILLED)
            cv2.putText(img,name,(x1+6,y2-6),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)

            # markAttendance(name)

            nameCheck.append(name)
            print(nameCheck)

            nameShow(name)
            
            # E = newTemp()

            # E = arduinoData.readline().decode().strip('\r\n')
       

            if len(nameCheck) == 5:
                    numName = nameCheck.count(name)
                    if numName == 5:
                        nameCheck.clear()
                        # print(E)
                        # check temp of the person
                        # tempTextShow()
                        # checkTemp()
                        isWhat = 0
                        while isWhat == 0:

                            E = arduinoData.readline().decode().strip('\r\n')
                            # TEMP > 35
                            if E == "1": 
                                print("High Temp")
                                # tCheck.append(E)
                                tCheck.clear()
                            elif E == "2":
                                print("No Reading")
                                # tCheck.append(E)
                                tCheck.clear()
                            elif E == "3":
                                # markAttendance(name)
                                tCheck.append(E)
                                print("Good to go")

                            if len(tCheck) == 25:
                                isWhat = 1
                                tCheck.clear()
                                markAttendance(name)
                                print("marked")
                            # print(tCheck)


                    else:
                        nameCheck.clear()

    cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    cv2.imshow('window', img)
    
    mList.clear()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    cv2.waitKey(1)
