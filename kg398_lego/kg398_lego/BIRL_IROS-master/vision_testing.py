import iros_vision_tools as ivt
import iros_vision_functions as ivfunc
import numpy as np
from matplotlib import pyplot as plt
import cv2
import copy
import os
PATH_TO_TASK_IMAGES = "task_images"
CAMERA = 1

def main():
    cali_img = cv2.imread("cali_img.jpg")
    circles_sorted, crop_points = ivt.run_calibration(cali_img, adjust=False)
    CAMERA = 1
    ivt.check_camera()
    cam_check = raw_input("Change Camera?: " )
    if cam_check == "yes":
        print "Current camera is "+str(CAMERA)
        CAMERA = raw_input("Which Camera to use?: ")
        CAMERA = int(CAMERA)
    
    
    while True:
        task = raw_input("Task?: ")
        if task == "calibrate":
            while True:
                ready = raw_input("Ready?: ")
                if ready == "yes":
                    cali_img = ivt.capture_pic(CAMERA,1)
                    circles_sorted, crop_points = ivt.run_calibration(cali_img)
                    
                    cv2.imwrite("cali_img.jpg",cali_img)
                    break
        if task == "1":
            task_img_1 = ivt.capture_pic(CAMERA,1)
            cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_1.jpg'), task_img_1)
            crop_task_img_1 = ivt.crop_out(task_img_1, crop_points)
            ivfunc.cup_saucer(crop_task_img_1, show=True)
            
            
            print "Done"

        if task=="2":
            task_img_2 = ivt.capture_pic(CAMERA,1)
            cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_2.jpg'), task_img_2)
            crop_task_img_2 = ivt.crop_out(task_img_2, crop_points)
            ivfunc.find_spoon(crop_task_img_2, show=True)
        


if __name__ == '__main__': main()
                
                