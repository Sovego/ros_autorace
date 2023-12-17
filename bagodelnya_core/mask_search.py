import cv2
import numpy as np


class MaskSerach():
    def __init__(self):
        cv2.namedWindow( "result" ) # создаем главное окно
        cv2.namedWindow( "settings" ) # создаем окно настроек

        cv2.createTrackbar('h1', 'settings', 0, 255, self.nothing)
        cv2.createTrackbar('s1', 'settings', 0, 255, self.nothing)
        cv2.createTrackbar('v1', 'settings', 0, 255, self.nothing)
        cv2.createTrackbar('h2', 'settings', 255, 255, self.nothing)
        cv2.createTrackbar('s2', 'settings', 255, 255, self.nothing)
        cv2.createTrackbar('v2', 'settings', 255, 255, self.nothing)
        self.crange = [0,0,0, 0,0,0]


    def __call__(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
 
        # считываем значения бегунков
        h1 = cv2.getTrackbarPos('h1', 'settings')
        s1 = cv2.getTrackbarPos('s1', 'settings')
        v1 = cv2.getTrackbarPos('v1', 'settings')
        h2 = cv2.getTrackbarPos('h2', 'settings')
        s2 = cv2.getTrackbarPos('s2', 'settings')
        v2 = cv2.getTrackbarPos('v2', 'settings')

        # формируем начальный и конечный цвет фильтра
        h_min = np.array((h1, s1, v1), np.uint8)
        h_max = np.array((h2, s2, v2), np.uint8)

        mask = cv2.inRange(hsv, h_min, h_max)
        mask = cv2.erode(mask, (3,3), iterations=2)
        mask = cv2.dilate(mask,(3,3),iterations=4)

        result = cv2.bitwise_and(frame,frame,mask=mask)


        cv2.imshow('Mask', mask)
        cv2.imshow('result', result) 


    def nothing(*arg):
        pass


if __name__ == "__main__":

    cap = cv2.VideoCapture("tests\\orig_runtime.mp4")
    # создаем 6 бегунков для настройки начального и конечного цвета фильтра


    #frame = cv2.imread("signs_images\\tunnel.png")

    while True:
        flag, frame = cap.read()
        if not flag:
            cap = cv2.VideoCapture("tests\\orig_runtime.mp4")
            continue
        
        #frame = cv2.resize(frame, (128,128))

        #frame = cv2.resize(frame,(1000,500))#убрать потом

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
    
        # считываем значения бегунков
        h1 = cv2.getTrackbarPos('h1', 'settings')
        s1 = cv2.getTrackbarPos('s1', 'settings')
        v1 = cv2.getTrackbarPos('v1', 'settings')
        h2 = cv2.getTrackbarPos('h2', 'settings')
        s2 = cv2.getTrackbarPos('s2', 'settings')
        v2 = cv2.getTrackbarPos('v2', 'settings')

        # формируем начальный и конечный цвет фильтра
        h_min = np.array((h1, s1, v1), np.uint8)
        h_max = np.array((h2, s2, v2), np.uint8)


        hsv = cv2.blur(hsv,(5,5))

        # накладываем фильтр на кадр в модели HSV
        mask = cv2.inRange(hsv, h_min, h_max)
        mask = cv2.erode(mask, (3,3), iterations=2)
        mask = cv2.dilate(mask,(3,3),iterations=4)

        result = cv2.bitwise_and(frame,frame,mask=mask)


        cv2.imshow('Mask', mask)
        cv2.imshow('result', result) 
        #cv2.imshow('HSV', hsv) 

        ch = cv2.waitKey(1)
        if ch == 27:
            break

    cap.release()
    cv2.destroyAllWindows()