import cv2
import numpy as np
import serial
import time
import os
import config

# Work-Flow
# 1. Receive trigger signal using UART
# 2. Open Camera
# 3. Capture Image
# 4. Crop Facial Area 
# 5. Save on Disk


def OnTrigger(storage_dir):
    # Init OpenCV Webcam
    cap = cv2.VideoCapture(config.CAMERA_INDEX)

    # Init Face Capturer
    face_cascade = cv2.CascadeClassifier(config.HAAR_DIR)

    done = False
    t0 = time.time()

    while not done:
        ret, frame = cap.read()
        # cv2.imshow('a', frame)
        # cv2.waitKey(1)
        if ret:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(frame_gray, 1.3, 5)

            for (x, y, w, h) in faces:
                if w >= (frame.shape[1]/10):
                    # cv2.rectangle(frame_gray, (x, y), (x+w, y+h), (255, 0, 0),  2)
                    # roi_gray = frame_gray[y:y+h, x:x+w]
                    # roi_color = frame[y:y+h, x:x+w]
                    
                    # Prepare file name rely on the time
                    t_arr = time.ctime()
                    fn_arr = t_arr.split(' ')
                    h_arr = fn_arr[-2].split(':')
                    filename = '{}_{}_{}_{}_{}_{}_{}.png'.format(fn_arr[0], fn_arr[1], fn_arr[2], h_arr[0], h_arr[1], h_arr[2], fn_arr[4])
                    filename = str(os.path.join(storage_dir, filename))

                    saved = cv2.imwrite(str(filename), frame)
                    if saved:
                        print('Saved image in {}'.format(filename))
                        done = True
                        cv2.destroyAllWindows()
        else:
            break

        # Timeout
        # Stop when cannot find any faces in TIMEOUT second
        if (int(time.time()) - int(t0)) >= config.TIMEOUT:
            break

    return done


if __name__ == '__main__':
    # Init UART
    port = serial.Serial(port=config.PORT, baudrate=config.BAURATE, timeout=1)
    
    # Check Port
    if port.isOpen():
        print('[LOG] - Connected with {}'.format(port))

    # Start
    while True:
        if port.isOpen():
            rev = port.readline().decode("utf-8")

            # When receiving signal
            if rev == config.RECEIVED_SIGNAL: 
                print('[LOG] - RECEIVED_SIGNAL')

                # Capture face
                ret = OnTrigger(config.STORAGE_DIR)
                
                # Return 2 if successful
                # Otherwise return 3 if failed
                if ret:
                    port.write(config.TRANSMITED_SUCCESS_SIGNAL)
                    print('[LOG] - TRANSMITED_SUCCESS_SIGNAL')
                else:
                    port.write(config.TRANSMITED_FAIL_SIGNAL)
                    print('[LOG] - TRANSMITED_FAIL_SIGNAL')
        else:
            break
    