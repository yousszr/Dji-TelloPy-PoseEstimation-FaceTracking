import cv2;
import imutils
import numpy as np

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml');
net = cv2.dnn.readNetFromCaffe('deploy.prototxt.txt', 'res10_300x300_ssd_iter_140000.caffemodel')
video = cv2.VideoCapture(0);

while True:
    check, frame = video.read();
    """convert frame to cv2 image and show"""
    frame = imutils.resize(frame, width=400)
 
    #CATTURO I FRAME E GLI CONVERTO IN BLOB
    (h, w) = frame.shape[:2]        
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
            (300, 300), (104.0, 177.0, 123.0))
    
    #passo i blob alla rete neurale per la identificazione delle faccie
    net.setInput(blob)
    detections = net.forward()
    
    
    #loop sulle faccie rilevate
    for i in range(0, detections.shape[2]):
            
            ##probabilità della predizione
            confidence = detections[0, 0, i, 2]

            # Filtra le faccie errate
            if confidence < 0.5:
                continue
            
            #calcolo  delle coordinate del box
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
       
        
            
           
            #disegno sul frame
            text = "{:.2f}%".format(confidence * 100)
            y = startY - 10 if startY - 10 > 10 else startY + 10
            cv2.rectangle(frame, (startX, startY), (endX, endY),
                (0, 0, 255), 2)
            
            
            cv2.putText(frame, text, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    
          

    cv2.imshow('tello', frame)
    _ = cv2.waitKey(1) & 0xFF

video.release();
cv2.destroyAllWindows();
