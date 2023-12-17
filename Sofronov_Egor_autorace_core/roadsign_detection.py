import cv2 as cv
import numpy as np
from imutils import paths


class SignDetector:
    """

    pedestrian_crossing_sign - 0
    traffic_construction - 1
    traffic_intersection - 2
    traffic_left - 3
    traffic_parking - 4 
    traffic_right - 5 
    tunnel - 6
    """

    def __init__(self, path_to_signs_imgs, crop_size=128, metrics=["ppa"], score_function=None, debug_mode=False):
        self.sign_templates_paths = sorted(list(paths.list_images(path_to_signs_imgs)))
        self.__templ_masks = dict(zip([path[13:-4] for path in self.sign_templates_paths],
                            [(0,0,180),
                             (0,47,153),
                             (0,35,158),
                             (0,0,226),
                             (0,0,205),
                             (0,0,226),
                             (0,114,182)])
                            )
        self.CROP_SIZE = crop_size
        self.debug_mode = debug_mode
        self.signs_templates = self.__load_templates()
        self.metrics = metrics
        self.n_templates = len(self.signs_templates)

        self.score_function = self.metrics[0] if score_function is None else score_function
        if self.score_function not in self.metrics:
            print("Error. qThe passed score_function is not in metrics list. \"metrics\" must include \"score_function\".")

        self.__dst_treshhold = 0.3

    def __call__(self, image, d_image):
        bbox, dst = self.detect(image, d_image) 
        classification = self.classify(image,bbox)

        if self.debug_mode:
            cv.imshow('Runtime', image)
            cv.waitKey(1)
        output = (dst[0], classification[self.score_function][0]) if dst[1] and classification[self.score_function][1]>=0.5 else (None,None)
        
        return output


    def greenlight_detect(self, image):
        #переводим в другой формат
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV )
        #блюрим
        hsv = cv.blur(hsv,(5,5))
        
        #Делаем маску
        mask = cv.inRange(hsv,(60,240,60),(255,255,255)) 
        #cv.imshow('Mask', mask) 

        #Убираем лишшние шумы на маске
        mask = cv.erode(mask, (3,3), iterations=2)
        mask = cv.dilate(mask,(3,3),iterations=4)
        #cv.imshow('Filtered mask', mask) 

        #вычисляем контуры
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

        if contours:
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            #отрисовка контура и баундинг бокса
            cv.drawContours(image, contours,0, (200,0,200),3)
            _, _ ,w,h = cv.boundingRect(contours[0])


            if self.debug_mode:
                cv.putText(image, f"Greenlight HW: {(h,w)}", (500, 370), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

            if w>=40 and h >=50:
                return 1

            


        return 0

    def __load_templates(self):
        #загружаем шаблоны знаков
        templates = []

        for path in self.sign_templates_paths:
            sign_image = cv.imread(path)

            assert sign_image is not None, "file could not be read, check with os.path.exists()"
            if self.debug_mode:
                print(f"Image at {path} has been successfully read.")

            sign_image = cv.resize(sign_image, (self.CROP_SIZE,self.CROP_SIZE))
            sign_image = cv.inRange(sign_image,self.__templ_masks[path[13:-4]],(255,255,255))
            templates.append(sign_image)
            #cv.imshow(path[13:-4], sign_image)

        return templates


    def detect(self, image, d_image):   
        #переводим в другой формат
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV )
        #блюрим
        hsv = cv.blur(hsv,(5,5))

        #Делаем маску
        mask = cv.inRange(hsv,(94,182,0),(255,255,255)) #(92,88,0)    (49,125,49)
        #cv.imshow('Mask', mask) 

        #Убираем лишшние шумы на маске
        mask = cv.erode(mask, (3,3), iterations=2)
        mask = cv.dilate(mask,(3,3),iterations=4)
        #cv.imshow('Filtered mask', mask) 

        #вычисляем контуры
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

        if contours:
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            #отрисовка контура и баундинг бокса
            #cv.drawContours(frame, contours,0, (200,0,200),3)
            x,y,w,h = cv.boundingRect(contours[0])
            cv.rectangle(image, (x,y), (x+w,y+h),(0,255,0),2)
            
            d_image_crop = d_image[y:y+h,x:x+w]#
            d_image_crop = cv.resize(d_image_crop, (self.CROP_SIZE, self.CROP_SIZE))

            dst = self.__is_valid_distance(d_image_crop)
            #print(dst)

            if self.debug_mode:
                dst_color = (0, 255, 0) if dst[1] else (255, 50, 50)
                cv.putText(image, f"Distance: {dst}", (500, 350), cv.FONT_HERSHEY_SIMPLEX, 0.5, dst_color, 2)
                cv.putText(image, f"Dst thold: {self.__dst_treshhold}", (500, 320), cv.FONT_HERSHEY_SIMPLEX, 0.5, dst_color, 2)
                #cv.imshow('a', image)
                cv.imshow('Detected depthcam',d_image_crop)
            

            return (x,y,w,h), dst 
        return (0,0,2,2), (None,False)


    def classify(self, image, bbox):
        x,y,w,h = bbox

        #кропаем задетектированный знак и считаем метрики с каждым из шаблонов
        image_crop = image[y:y+h,x:x+w]
        image_crop = cv.resize(image_crop, (self.CROP_SIZE,self.CROP_SIZE))       
        image_crop = cv.inRange(image_crop, (0,30,101), (255,255,255)) #0,10,101

        if self.debug_mode:
            cv.imshow('Detected', image_crop)

        metric_values = self.calc_metrics(image_crop)
        classification = {}

        for idx, metric in enumerate(metric_values):
            max_idx = np.argmax(metric_values[metric])

            classification[metric] = (max_idx,metric_values[metric][max_idx])

            if self.debug_mode:
                if metric_values[metric][max_idx]<0.5:
                        cv.putText(image, f"nothing ({metric})", (0, (idx+1)*14), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        #print("nothing")
                else:  
                    #pass
                    cv.putText(image, f"{list(self.__templ_masks.keys())[max_idx]}: {classification[metric][1]:.4%} ({metric})", (0, (idx+1)*14), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #print(classification[metric][0],classification[metric][1])


        return classification


    def calc_metrics(self, image):
        metric_values = {metric : np.zeros((self.n_templates,)) for metric in self.metrics}
        score = 0
        for idx, template in enumerate(self.signs_templates):
            for metric in metric_values:
                match metric:
                    case "ppa":
                        score = self.ppa(image, template)

                    case "dice":
                        score = self.dice(image, template)

                    case "iou":
                        score = self.iou(image, template)

                    case _:
                        print("Error: Invalid metric name. Exit.")
                        exit()

                metric_values[metric][idx] = score

        return metric_values


    def ppa(self, image, template):
        total_pixels = image.size
        matching_pixels = np.sum(image==template)
        score = matching_pixels/total_pixels
        return score


    def dice(self, image, template):
        intersection = cv.bitwise_and(image, template)
        score = 2* cv.countNonZero(intersection) / (cv.countNonZero(image)+cv.countNonZero(template))
        return score


    def iou(self, image, template):
        intersection = cv.bitwise_and(image, template)
        union = cv.bitwise_or(image, template)
        score = cv.countNonZero(intersection) / cv.countNonZero(union)
        return score


    def __is_valid_distance(self, image):
        #image = cv.split(image)[0] #убрать потом
        #image = image*100
        #image = image[np.isnan(image)]
        image = image[np.isfinite(image)]
        img_mean = image.mean()
        #print(image)
        #img_mean=image[self.CROP_SIZE//2,self.CROP_SIZE//2] 
        out = (img_mean,True) if img_mean < self.__dst_treshhold else (img_mean,False)
        
        return out


    def set_dst_treshhold(self, value):
        self.__dst_treshhold = value


if __name__ == "__main__":
    VIDEO_PATH = "tests\\orig_runtime.mp4"
    DEPTH_PATH =  "tests\\orig_runtime_deep.mp4"

    cap = cv.VideoCapture(VIDEO_PATH)
    depth_cap=cv.VideoCapture(DEPTH_PATH)

    detector = SignDetector(path_to_signs_imgs="signs_images",debug_mode=True)


    while True:
        ret, frame = cap.read()
        if not ret:
            cap = cv.VideoCapture(VIDEO_PATH)
            continue

        d_ret, d_frame = depth_cap.read()
        if not d_ret:
            depth_cap = cv.VideoCapture(DEPTH_PATH)
            continue

        detect_result = detector(frame, d_frame)

        print(detect_result)

        key = cv.waitKey(1) & 0xff
        if key == ord('q'):
            break
