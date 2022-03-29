import imp
from time import sleep
import mediapipe as mp
import cv2
import numpy as np
import uuid
import os

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0, cv2.CAP_V4L)

with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        h, w, c = frame.shape

        # BGR 2 RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Flip on horizontal
        image = cv2.flip(image, 1)

        # Set flag
        image.flags.writeable = False
        
        # Detection
        results = hands.process(image)
        
        # Set flag 2 true
        image.flags.writeable = True
        
        # RGB 2 BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # DISTINGUISH BETWEEN RIGHT AND LEFT HANDS
        #myHands=[]
        handsType=[]

        # Rendering results
        if results.multi_hand_landmarks:

            # 尋找影像中的手 handsType=['Right', 'Left'] 先偵測到左手再右手（沒寫錯，別懷疑）
            for hand in results.multi_handedness:
                handType=hand.classification[0].label
                handsType.append(handType)


            if 'Right' in handsType[0] and 'Left' in handsType[1]:

                #------------------------ 左手 加入 joint, 框框 ----------------------------------------------------------
                count=0
                for handLMs in results.multi_hand_landmarks: #會進 for迴圈兩次 第一次會先找到右手 第二次會先找到左手
                        
                    x_max = 0
                    y_max = 0
                    x_min = w
                    y_min = h
                    
                    for lm in handLMs.landmark: # 第一圈會先找右手的 21個點座標 第二圈會先找左手的 21個點座標

                        # 找到 21個點座標（x,y）的 x_max ,x_min ,y_max ,y_min
                        x, y = int(lm.x * w), int(lm.y * h)
                        if x > x_max:
                            x_max = x
                        if x < x_min:
                            x_min = x
                        if y > y_max:
                            y_max = y
                        if y < y_min:
                            y_min = y

                    if count == 1: #我不希望偵測右手 所以加入 count 當counr=1時，才可以繼續往下
                        
                        # 將21個點座標 放到影響上
                        mp_drawing.draw_landmarks(image, handLMs, mp_hands.HAND_CONNECTIONS,
                                                #mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                                mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                                )

                        text = 'Left'
                        cv2.putText(image, text, (x_min-20, y_min-50), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 255), 1, cv2.LINE_AA)
                        cv2.rectangle(image, (x_min-20, y_min-20), (x_max+20, y_max+20), (255, 0, 0), 2)
                        handsType[1] = 'Right'
                                    
                    count+=1 #第一圈結束  count＋1
                
                #record y_min_left
                y_min_left = y_min-20
                
                #------------------------ 右手 加入 joint, 框框 ----------------------------------------------------------
                if 'Right' in handsType[0]:
                    
                    for handLMs in results.multi_hand_landmarks:
                        
                        x_max = 0
                        y_max = 0
                        x_min = w
                        y_min = h
                        for lm in handLMs.landmark:
                            
                            x, y = int(lm.x * w), int(lm.y * h)
                            if x > x_max:
                                x_max = x
                            if x < x_min:
                                x_min = x
                            if y > y_max:
                                y_max = y
                            if y < y_min:
                                y_min = y

                            mp_drawing.draw_landmarks(image, handLMs, mp_hands.HAND_CONNECTIONS,
                                                    #mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                                    mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                                    )

                        if 'Right' in handsType[0]:
                            text = 'Right'
                            cv2.putText(image, text, (x_min-20, y_min-50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 255), 1, cv2.LINE_AA)
                            cv2.rectangle(image, (x_min-20, y_min-20), (x_max+20, y_max+20), (0, 0, 255), 2)
                            handsType[0] = 'Left'
                            break

                #record y_min_right
                y_min_right = y_min-20

                #y_min_left, y_min_right相減
                print(y_min_left-y_min_right)


            elif 'Right' in handsType[0]:
                print("Right")
            elif 'Left' in handsType[0]:
                print("Left")


                   

        image = cv2.resize(image, (1280, 960), interpolation=cv2.INTER_AREA)
        cv2.imshow('Hand Tracking', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()