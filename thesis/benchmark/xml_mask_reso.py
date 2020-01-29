#!/usr/bin/env python
import numpy as np
import os
import cv2
import csv
import xmltodict
import random
from argparse import ArgumentParser
WIDTH = 1280
HEIGHT = 720

"""
Revised by Yung-Shan Su on 1/30, 2019
Used to convert Labelme xml file to 1-channel mask image.
"""
brand_list = ['background','crayons','vanish','milo','raisins','andes','pocky','stax','hunts','3m','dove','pringles','nutella','domino',\
        '90_crayons','90_vanish','90_milo','90_raisins','90_andes','90_pocky','90_stax','90_hunts','90_3m','90_dove','90_pringles','90_nutella','90_domino',\
        '180_crayons','180_vanish','180_milo','180_raisins','180_andes','180_pocky','180_stax','180_hunts','180_3m','180_dove','180_pringles','180_nutella','180_domino',\
        '270_crayons','270_vanish','270_milo','270_raisins','270_andes','270_pocky','270_stax','270_hunts','270_3m','270_dove','270_pringles','270_nutella','270_domino','barcode']
#product_list = set(['folgers','crayola','viva','vanish','milo','swissmiss','cocacola','raisins','mm','andes','pocky','kotex','macadamia','stax','kellogg','hunts','3m','heineken','libava','barcode'])
parser = ArgumentParser()
parser.add_argument("-d", "--datapath", dest="datapath")
args = parser.parse_args()
count = 0
def xml2mask(xml_path):
    """Convert xml file to a label image
    Read a xml file, and generate a gray image.
    There are only three different values in a image:
    255: object
    128: barcode
    0: background
    Args:
        xml_path: xml path
    Returns:
        1-channel lable image
    """
    global product_list, count
    file_exist = os.path.isfile(xml_path)    # True
    mask = np.zeros([HEIGHT, WIDTH], dtype = np.uint8)
    if file_exist:
        # print xml_path
        with open(xml_path) as fd:
            label_dict = xmltodict.parse(fd.read())

    else:
        print "This path doesn't exist"
        # print xml_path
        return mask
    if 'object' in label_dict['annotation']:
        if type(label_dict['annotation']['object']).__name__ != "list":
            try:
                object_ = label_dict['annotation']['object']
                if  object_['name'] in brand_list and object_['deleted'] == '0':
                    poly_vertice = []
                    for pts_idx in object_['polygon']['pt']:
                        poly_vertice.append([int(pts_idx['x']), int(pts_idx['y'])])
                    poly_vertice = np.array(poly_vertice, np.int32)
                    for i in range(len(brand_list)):
                        if brand_list[i] == object_['name']:
                            cv2.fillConvexPoly(mask, poly_vertice, i*4)
                # if object_['name'] == "barcode":
                #     poly_vertice = []
                #     for pts_idx in object_['polygon']['pt']:
                #         poly_vertice.append([int(pts_idx['x']), int(pts_idx['y'])])
                #     poly_vertice = np.array(poly_vertice, np.int32)
                #     cv2.fillConvexPoly(mask, poly_vertice, 128)
            except Exception as e:
                print (e)
        else:
            try:
                for object_ in label_dict['annotation']['object']:
                    # print (object_['name'])
                    if  object_['name'] in brand_list and object_['deleted'] == '0':
                        count += 1
                        poly_vertice = []
                        for pts_idx in object_['polygon']['pt']:
                            poly_vertice.append([int(pts_idx['x']), int(pts_idx['y'])])
                        poly_vertice = np.array(poly_vertice, np.int32)
                        for i in range(len(brand_list)):
                            if brand_list[i] == object_['name']:
                                cv2.fillConvexPoly(mask, poly_vertice, i*4)
                    # if object_['name'] == "barcode":
                    #     poly_vertice = []
                    #     for pts_idx in object_['polygon']['pt']:
                    #         poly_vertice.append([int(pts_idx['x']), int(pts_idx['y'])])
                    #     poly_vertice = np.array(poly_vertice, np.int32)
                    #     cv2.fillConvexPoly(mask, poly_vertice, 128)
            except Exception as e:
                print (e)
    return mask


def main(args):
    # imagefolder_path = "/home/michael/Downloads/mini_competition_dataset/image"
    datapath = args.datapath
    xmlfolder_path = os.path.join(datapath, "xml")
    maskfolder_path = os.path.join(datapath, "mask")

    if not os.path.isdir(maskfolder_path):
        os.makedirs(maskfolder_path)



    products = os.listdir(xmlfolder_path)
    for product in products:
        print ("Process " + product)
        product_path = os.path.join(xmlfolder_path, product)

        mask_product_path = os.path.join(maskfolder_path, product)
        if not os.path.isdir(mask_product_path):
            os.makedirs(mask_product_path)


        scenes = os.listdir(product_path)
        for scene in sorted(scenes):
            scene_path = os.path.join(product_path, scene)
            mask_scene_path = os.path.join(mask_product_path, scene)

            # print scene_path
            if not os.path.isdir(mask_scene_path):
                os.makedirs(mask_scene_path)

            xml_filenames = os.listdir(scene_path)
            for xml_filename in sorted(xml_filenames):

                file_path = os.path.join(scene_path, xml_filename)
                mask = xml2mask(file_path)
                mask_file_name = xml_filename.split("xml")[0] + "png"
                save_path = os.path.join(mask_scene_path, mask_file_name)
                # print np.shape(mask)
                cv2.imwrite(save_path, mask,[int(cv2.IMWRITE_JPEG_QUALITY), 100])
                image_file_path = save_path.replace("mask", "image")
                image_file_path = image_file_path.replace("png","jpg")
    
    print count
if __name__ == "__main__":
    main(args)
    