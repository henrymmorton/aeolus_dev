import numpy as np
import cv2
import argparse

def vis_seg_label(img_path):
    label_img = cv2.imread(img_path)
    image_dims = label_img.shape
    for row in range(image_dims[0]):
        for column in range(image_dims[1]):
            if label_img[row, column, 0] == 1:
                label_img[row, column] = np.array([255, 255, 255])
            

    cv2.imshow("segmentation_label", label_img)

    cv2.waitKey(0) 
  
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_path', required=True, help="The path to the image you want to visualize")
    args = parser.parse_args()
    image_path = args.image_path
    vis_seg_label(image_path)