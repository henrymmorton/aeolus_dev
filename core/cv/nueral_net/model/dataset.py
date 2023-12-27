import numpy as np
import os
import re

from PIL import Image

from torch.utils.data import Dataset

EXTENSIONS = ['.jpg', '.png']

def load_image(file):
    return Image.open(file)

def is_image(filename):
    return any(filename.endswith(ext) for ext in EXTENSIONS)

def image_path(root, basename, extension):
    return os.path.join(root, f'{basename}{extension}')

def image_path_city(root, name):
    return os.path.join(root, f'{name}')

def image_basename(filename):
    return os.path.basename(os.path.splitext(filename)[0])


def parse_gt_file(gt_filepath):
    with open(gt_filepath,"r") as gt_file:
        gt_list = []
        for image_pair in gt_file:
            paths_truths = re.split("\s", image_pair)
            image_path = paths_truths[0]
            label_path = paths_truths[1]
            lane_truths = paths_truths[2:]
            image_pair_tuple = (image_path, label_path, lane_truths)
            gt_list.append(image_pair_tuple)
    
    return gt_list
            

class TuSimple(Dataset):

    def __init__(self, root, co_transform = None):
        self.root = root
        self.co_transform = co_transform
        self.images_root = os.path.join(root, 'clips')
        self.labels_root = os.path.join(root, 'segmentation_test')
        gt_association_file = os.path.join(root, 'segmentation_test/list/train_val_gt.txt')
        self.gt_list = parse_gt_file(gt_association_file)

    def __len__(self):
        return len(self.gt_list)

    def __getitem__(self, index):
        # Get the image at the index
        image_leaf = self.gt_list[index][0]
        image_leaf = image_leaf[1:]
        image_path = os.path.join(self.root, image_leaf)
        image = load_image(image_path)

        # Get the corresponding label
        label_leaf = self.gt_list[index][1]
        label_leaf = label_leaf[1:]
        label_path = os.path.join(self.root, label_leaf)
        label = load_image(label_path)

        print("original image dimensions are: ", image.size)
        # print("original label dimensions are: ", label.size)

        if self.co_transform is not None:
            image, label = self.co_transform(image, label)


        print("altered image_dimensions are: ", image.shape)
        # print("altered label_dimensions are: ", label.shape)

        return image, label