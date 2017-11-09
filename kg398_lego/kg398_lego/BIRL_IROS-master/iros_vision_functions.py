import iros_vision_tools as ivt
import numpy as np
import copy
import os
import cv2
from scipy.spatial.distance import cdist
from matplotlib import pyplot as plt

import scipy



def cup_saucer(test_image, show=False):
    test_img = copy.copy(test_image)
        
    CAL_PARAM = {'thresh': [25, 60],
            'radius': [30,85]}
    circles, cimg = ivt.find_circles(copy.copy(test_img), 6, param=CAL_PARAM, blur=1, show=False)

    CAL_PARAM = {'thresh': [105, 140],
                'radius': [30,85]}
    circles2, cimg = ivt.find_circles2(copy.copy(test_img), 2, param=CAL_PARAM, blur=1, overlap=True, separation=180, show=False)

    print len(circles[0])
    print len(circles2[0])

    table_circles = {}
    store = []
    for j,i in enumerate(circles2[0]):
        circle_info = {}
        circle_info["id"] = j
        coords = np.array([i[:-1]])
        radius = i[-1]
        circle_info["circle"]=i
        num = 0
        for k in (circles[0]):
            #print np.array([k[:-1]])
            if cdist(np.array([k[:-1]]),coords)<radius:
                num = num+1
        circle_info["num_circles"]=num
        store.append(circle_info)
    
    for member in store:
        if member["num_circles"]>3:
            color = (0,255,0)
            table_circles["saucer"]=member
        else:
            color = (0,0,255)
            table_circles["mug"]=member
        print member['circle']

        cv2.circle(cimg,(int(member["circle"][0]),int(member["circle"][1])),int(member["circle"][2]),color,1)
                    # draw the center of the ci~rcle
        cv2.circle(cimg,(int(member["circle"][0]),int(member["circle"][1])),2,color,1)
    if show==True:
        plt.imshow(cimg)
        plt.show()

    return table_circles

def cup_saucer2(test_image, show=False):
    test_img = copy.copy(test_image)

    CAL_PARAM = {'thresh': [105, 140],
                'radius': [25,55]}
    
    circles2, cimg = ivt.find_circles2(copy.copy(test_img), 2, param=CAL_PARAM, blur=1, overlap=False, 
                                       separation=80, show=False)
    
    print len(circles2[0])
    
    

    table_circles = {}
    store = []
    for j,i in enumerate(circles2[0]):
        circle_info = {}
        circle_info["id"] = j
        coords = np.array([i[:-1]])
        
        radius = i[-1]
        circle_info["radius"] = radius
        circle_info["circle"]=i
        num = 0
        store.append(circle_info)
    
    if store[0]["radius"]>store[0]["radius"]:
        table_circles["saucer"]=store[0]
        table_circles["mug"]=store[1]
    else:
        table_circles["mug"]=store[0]
        table_circles["saucer"]=store[1]
    
    cv2.circle(cimg,(int(table_circles["saucer"]["circle"][0]),
                     int(table_circles["saucer"]["circle"][1])),
               int(table_circles["saucer"]["circle"][2]),
               (0,255,0),1)
                # draw the center of the ci~rcle
    cv2.circle(cimg,(int(table_circles["mug"]["circle"][0]),
                     int(table_circles["mug"]["circle"][1])),
               int(table_circles["mug"]["circle"][2]),
               (0,0,255),1)
    if show==True:
        plt.imshow(cimg)
        plt.show()

    return table_circles

def find_spoon(image, show=True):
    img = copy.copy(image)
    edged, edg_img, cnts, hierarchy=ivt.extract_contours(copy.copy(img), 
                                                        min_thresh=25, 
                                                        max_thresh=200, 
                                                        blur = 3, dilate=4, erode=1, 
                                                        cnt_mode = cv2.RETR_TREE)
    CAL_PARAM = {'thresh': [75, 100],
                 'radius': [25,35]}
    
    minsize=1
    mindistance = 1000
    box_minsize = 1
    
    circles, cimg = ivt.find_circles2(copy.copy(img), 1, param=CAL_PARAM, blur=1, show=False)

    show_img = copy.copy(img)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_RGB2GRAY)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_GRAY2RGB)
    
    for cnt in cnts:
        current_outer_contour = []
        for points in cnt:
            current_outer_contour.append(points[0])

        distance = cdist(np.array([[circles[0][0][0],circles[0][0][1]]]),current_outer_contour)

        if cv2.contourArea(cnt) < minsize:
            print("Object at #{} REJECTED because CONTOUR not big enough: ".format(cnt[0]), cv2.contourArea(cnt))
            continue

        if distance[0][0] > mindistance:
            print("Object at #{} REJECTED because not CLOSE ENOUGH: ".format(cnt[0]), distance[0][0])
            continue
        mindistance = distance[0][0]

        box = ivt.extract_minBox(cnt)
        box_area = abs((box[0][0]-box[2][0])*(box[1][1]-box[0][1]))
        
        if box_area < box_minsize:
            print("Object at #{} REJECTED because BOX not big enough: ".format(cnt[0]), box_area)
            continue
        fnode = ivt.farthest_node([circles[0][0][0],circles[0][0][1]], current_outer_contour)
    
    print "CIRCLE:", circles
    cv2.circle(show_img,(int(circles[0][0][0]),int(circles[0][0][1])),3,(0,255,0),5)
    cv2.circle(show_img, (fnode[0][0], fnode[0][1]), 3, (0,255,255),5)
    if show:
        plt.figure("Spoon and Cup")
        plt.subplot(2,2,1)
        plt.imshow(show_img)
        cv2.imwrite("show_img.jpg",show_img)
        plt.subplot(2,2,2)
        plt.imshow(cimg)
        plt.subplot(2,2,3)
        plt.imshow(edged)
        plt.show()
    #print "FNODE: ", fnode
    
    

    return circles[0], fnode 
from numpy import linalg

def extract_shape_contours(image, threshold=120, show=True):
    img = copy.copy(image)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    gray_flat = gray[np.isfinite(gray)]
    black, white = np.min(gray_flat), np.max(gray_flat)
    if show:
        plt.figure('Gray Image and Histogram')
        
        plt.subplot(1, 2, 1)
        plt.imshow(img)
        plt.title('Gray Image')
        
        plt.subplot(1,2,2)
        plt.hist(gray_flat, bins=np.linspace(black,white, 20), histtype='step')
        plt.xlim(black, white)
        plt.title('Gray Histogram')
        plt.xlabel('Value')
        plt.ylabel('Count')
        plt.show()
        
    gray_mask = np.zeros_like(gray)
    gray_mask[gray<threshold] = 250
    gray_mask[-20:-1,:] = 0
    gray_mask[:, 0:30] = 0
    gray_mask[:, -20:-1] = 0
    
    
    edged, edg_img, cnts, hierarchy=ivt.extract_contours(gray_mask, 
                                                         min_thresh=60, max_thresh=250, 
                                                         blur=3, dilate=1, erode=1, 
                                                         cnt_mode=cv2.RETR_EXTERNAL)
    
    if show:
        plt.figure("Extracted Shapes")
        plt.subplot(1,2,1)
        plt.imshow(gray_mask)
        plt.subplot(1,2,2)
        plt.imshow(edged)
        plt.show()
    cv2.imwrite("edged_img.jpg", edged)
    cv2.imwrite("gray_mask.jpg", gray_mask)
    return cnts, hierarchy

def extract_shape_list(image, threshold, show=True):
    cnts, hierarchy = extract_shape_contours(image, threshold=threshold, show=show)
    
    show_img = copy.copy(image)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_RGB2GRAY)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_GRAY2RGB)
    shape_list = []
    
    for i,c in enumerate(cnts):
        shape_object = {}
        
        if hierarchy[0][i][3]!=-1:
            print "Not External shape"
            continue
        
        if cv2.contourArea(c) < 150:
            print "Too small"
            continue
        
        approx = detect(c)
        cmax = np.max(approx, axis = 0)
        cX, cY = cmax[0][0], cmax[0][1]

        peri = cv2.arcLength(c, True)
        if peri>300:
            print "Perimeter too big: ", peri
            continue
        
        cv2.drawContours(show_img, [approx], -1, (0,255,0), 1)
        
        # Find Shape Centre
        centre = np.mean(approx, axis=0)
        shape_object['centre']=centre

        if len(approx)==3:
            shape=2
            point1 = approx[0]
            point2 = (approx[1]+approx[2])/2
            direction = (point1-point2)/linalg.norm(point1-point2)
        elif len(approx)==4:
            vect1 = approx[0]-approx[1]
            vect2 = approx[1]-approx[2]
            
            len_side1 = linalg.norm(vect1)
            len_side2 = linalg.norm(vect2)
            long_side, short_side = 0,0
            if len_side1 > len_side2:
                long_side = len_side1
                short_side = len_side2
                direction = vect1/linalg.norm(vect1)
                point1 = (approx[1]+approx[2])/2
                point2 = (approx[3]+approx[0])/2
            else:
                long_side = len_side2
                short_side = len_side1
                direction = vect2/linalg.norm(vect2)
                point1 = (approx[0]+approx[1])/2
                point2 = (approx[2]+approx[3])/2

            aspect = float(long_side)/short_side

            if aspect < 1:
                aspect = 1/aspect
            if aspect>1.4:
                shape = 1
            else:
                shape = 3
                
        elif len(approx) == 5:
            shape = 4
            point1 = approx[0]
            point2 = (approx[2]+approx[3])/2
            direction = (point2-point1)/linalg.norm(point1-point2)
        else:
            shape = 0
            radius = centre - approx[0]
            direction = radius/linalg.norm(radius)
            point1 = centre - radius
            point2 = centre + radius
            
        dir_rat = direction[0][1]/-direction[0][0]
        angle = np.arctan(dir_rat)*180/np.pi
        point1 = point1-direction*8
        
        shape_object['ratio']=dir_rat
        shape_object['angle'] = angle
        shape_object['shape']=shape
        shape_object['approx']=approx
        shape_object['point1']=point1
        shape_object['point2']=point2
        cv2.circle(show_img, (int(point1[0][0]), int(point1[0][1])), 2, (0,255,255),2)
        cv2.circle(show_img, (int(point2[0][0]), int(point2[0][1])), 2, (255,0,255),2)
        cv2.putText(show_img, str(shape), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        shape_list.append(shape_object)
    cv2.imwrite("labelled_shape.jpg",show_img)
    if show:
        plt.figure("Labelled Shapes")
        plt.imshow(show_img)
        plt.show()
    return shape_list
                
def detect(c):
    # Approximate Shape
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.03 * peri, True)
    return approx

def find_spoon2(image, show=True):
    img = copy.copy(image)
    
    CAL_PARAM = {'thresh': [75, 100],
                 'radius': [25,35]}
    
    CROP_RADIUS = 80
    PADDING = 30
    
    circles, cimg = ivt.find_circles2(copy.copy(img), 2, param=CAL_PARAM, blur=1, show=False)
    print "CIRCLES: ", circles
    empty_cup_centre, empty_cup_id = ivt.farthest_node(np.array([180, 185]),
                                                    np.array([[circles[0][0][0], circles[0][0][1]],
                                                     [circles[0][1][0], circles[0][1][1]]]))
    print "EMPTY_MUG_DISTANCE: ", np.array(empty_cup_centre)
    
    empty_circles = np.array([[circles[0][empty_cup_id]]])
    spoon_circles = np.array([[circles[0][1-empty_cup_id]]])  
    
    sx, sy = int(spoon_circles[0][0][0]), int(spoon_circles[0][0][1])
    print "SPOON_MUG_WORLD_CENTRE: ", sx, sy
    
    #img_3b = ivt.black_out(copy.copy(crop_task_img_3), [180,-1,0,-1])
    img_3b = copy.copy(img)
    
    img_3b[sy-CROP_RADIUS+PADDING:sy+CROP_RADIUS-PADDING, sx-CROP_RADIUS+PADDING:sx+CROP_RADIUS-PADDING]=[0,0,0]
    
    print "EMPTY_CUP_WORLD: ",empty_circles
    ex, ey, er = int(empty_circles[0][0][0]), int(empty_circles[0][0][1]), int(empty_circles[0][0][-1]+1)
    
    r,g,b = cv2.split(img)
    
    img_3a = copy.copy(img)
    img_3a[ey-er:ey+er,ex-er:ex+er] = [r.mean(), g.mean(), b.mean()]
    img_3a = img_3a[sy-CROP_RADIUS:sy+CROP_RADIUS, sx-CROP_RADIUS:sx+CROP_RADIUS]

    
    mug_centre = np.array([[sy,sx]])
    cropped_mug_centre = np.array([[CROP_RADIUS, CROP_RADIUS]])
    
    print mug_centre
    print cropped_mug_centre
    
    edged, edg_img, cnts, hierarchy=ivt.extract_contours(copy.copy(img_3a), 
                                                        min_thresh=65, 
                                                        max_thresh=240, 
                                                        blur = 5, dilate=3, erode=2, 
                                                        cnt_mode = cv2.RETR_TREE)
    
    

    minsize=0
    mindistance = 190
    box_minsize = 0
    minperi = 40
    

    
    show_img = copy.copy(img)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_RGB2GRAY)
    show_img = cv2.cvtColor(show_img, cv2.COLOR_GRAY2RGB)
    fnode = np.array([CROP_RADIUS, CROP_RADIUS])
    
    for cnt in cnts:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        current_outer_contour = []
        for points in approx:
            current_outer_contour.append(points[0])

        distance = cdist(np.array(cropped_mug_centre),current_outer_contour)
        print "CURRENT", np.shape(current_outer_contour)

        if cv2.contourArea(cnt) < minsize:
            print("Object at #{} REJECTED because CONTOUR not big enough: ".format(cnt[0]), cv2.contourArea(cnt))
            continue
            
        if peri < minperi:
            print("Object at #{} REJECTED because PERIMETER not long enough: ".format(cnt[0]), peri)
            continue
            

        if distance[0][0] > mindistance:
            print("Object at #{} REJECTED because not CLOSE ENOUGH: ".format(cnt[0]), distance[0][0])
            continue
        #mindistance = distance[0][0]

        box = ivt.extract_minBox(cnt)
        box_area = cv2.contourArea(np.array([box]))
        #box_area = abs((box[0][0]-box[2][0])*(box[1][1]-box[0][1]))
        
        if box_area < box_minsize:
            print("Object at #{} REJECTED because BOX not big enough: ".format(cnt[0]), box_area)
            continue
            
        print("Object at #{} ACCEPTED: ".format(cnt[0]))
        print "    Contour Area: ",cv2.contourArea(cnt)
        print "    Perimeter:    ",peri
        print "    Distance:     ",distance[0][0]
        print "    Box Area:     ",box_area
        
        fnode_test, fnode_test_id = ivt.farthest_node(cropped_mug_centre[0],  current_outer_contour)
        # Make edge more accurate
        fnode_testa = current_outer_contour[fnode_test_id+1]
        fnode_testb = current_outer_contour[fnode_test_id-1]
        
        fnode_test_dist = cdist(np.array(cropped_mug_centre),np.array([fnode_test]))
        
        fnode_add = fnode_test
        fnode_num = 1
        if fnode_test_dist - cdist(np.array(cropped_mug_centre),np.array([fnode_testa]))<2:
            print "HELLO"
            print fnode_test_dist - cdist(np.array(cropped_mug_centre),np.array([fnode_testa]))
            fnode_add = fnode_add + fnode_testa
            fnode_num = fnode_num+1
        
        if fnode_test_dist - cdist(np.array(cropped_mug_centre),np.array([fnode_testb]))<2:
            print "HELLO2"
            print fnode_test_dist - cdist(np.array(cropped_mug_centre),np.array([fnode_testb]))
            fnode_add = fnode_add + fnode_testb
            fnode_num = fnode_num+1
            
        fnode_test_mean = fnode_add/fnode_num
        print "FNODE MEAN: ", fnode_test_mean,
        print "FNODE ORIG: ", fnode_test 
        fnode_dist = cdist(np.array(cropped_mug_centre),np.array([fnode_test_mean]))
        fnode_dist2 = cdist(np.array(cropped_mug_centre),np.array([fnode]))
        print "FNODE_DISTANCES: ", fnode_dist, fnode_dist2,
        if fnode_dist > fnode_dist2:
            fnode = fnode_test_mean
        print "FNODE: ", fnode
        
    print "CIRCLE:", spoon_circles
    spoon_mug = np.array([sx,sy])
    #spoon_edge = fnode - (fnode-spoon_mug)*
    spoon_edge = fnode
    if cdist(np.array(cropped_mug_centre), np.array([spoon_edge])) < 60:
        print cropped_mug_centre
        print spoon_edge
        print "VECTOR!!!!", cdist(np.array(cropped_mug_centre), np.array([spoon_edge]))
        fvect = spoon_edge-CROP_RADIUS
        unit_fvect = fvect/scipy.linalg.norm(fvect)
        spoon_edge = unit_fvect*70 + 80
        
    print "SPOON_EDGE: ", spoon_edge
    
    spoon_mug = np.array([sx,sy])
    fnode_world = np.array([sx-CROP_RADIUS+fnode[0], sy-CROP_RADIUS+fnode[1]])
    spoon_edge_world = [sx-CROP_RADIUS+int(spoon_edge[0]), sy-CROP_RADIUS+int(spoon_edge[1])]
    
    
    cv2.circle(show_img,(int(spoon_mug[0]), int(spoon_mug[1])),3,(0,255,0),5)
    cv2.circle(show_img, (int(fnode_world[0]), int(fnode_world[1])), 3, (0,255,255),5)
    cv2.circle(show_img, (int(spoon_edge_world[0]), int(spoon_edge_world[1])), 3, (255,0,255),5)
    if show:
        plt.figure("Spoon and Cup", figsize = [9,9])
        plt.subplot(2,2,1)
        plt.imshow(show_img)
        cv2.imwrite("show_img.jpg",show_img)
        plt.subplot(2,2,2)
        plt.imshow(cimg)
        plt.subplot(2,2,3)
        plt.imshow(edged)
        plt.subplot(2,2,4)
        plt.imshow(img_3a)
        plt.show()
    #print "FNODE: ", fnode
    
    return spoon_mug, spoon_edge_world, empty_cup_centre
