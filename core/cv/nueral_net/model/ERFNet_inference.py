from cv2 import imwrite

NUM_CLASSES = 1

def input_transform(image):
    print(image.size)
    print(image.mode)
    print(type(image))
     
    if (image.mode == 'RGB'):
        print('changing from rgb to bgr')
        imagenp = np.asarray(image)
        imagenp = imagenp[:, :, ::-1]
        image = Image.fromarray(imagenp)

    if (image.size != (720, 1280)):
        print("resizing image")
        image = Resize((720, 1280),Image.BILINEAR)(image)
    
    image = ToTensor()(image)
    
    return image

class ThresholdTransform(object):
  def __init__(self, thresh):
    self.thresh = thresh

  def __call__(self, x):
    thresholded = x > self.thresh
    scaled = thresholded * 255
    return scaled

def main(args):

    # Path to trained model state dict
    weightspath = args.loadDir + '/' + args.loadWeights

    print ("Loading model")
    print ("Loading weights: " + weightspath)

    model = ERFNet(NUM_CLASSES)
  
    model = torch.nn.DataParallel(model)
    if (not args.cpu):
        model = model.cuda()
    
    if (args.cpu):
        checkpoint_file = torch.load(weightspath, map_location=torch.device('cpu'))
    else:
        checkpoint_file = torch.load(weightspath)
    state_dict = checkpoint_file['state_dict']
    model.load_state_dict(state_dict)
    
    print ("Model and weights LOADED successfully")

    # Puts the model in evaluation mode
    model.eval()

    if(not os.path.exists(args.imagedir)):
        print ("Error: image could not be loaded")
        
    image = Image.open(args.imagedir)
    
    image = input_transform(image)
    
    image = image[None, :]
    
    print(image.size())

    if (not args.cpu):
        image = image.cuda()

    with torch.no_grad():
        outputs = model(image)
        sigmoid = torch.nn.Sigmoid()
        outputs = sigmoid(outputs)
        output_image = ThresholdTransform(0.5)(outputs)

    image = image.cpu().numpy()[0]
    output_image = output_image.cpu().numpy()[0]

    image = np.transpose(image, (1, 2, 0))
    image = image * 255
    output_image = np.transpose(output_image, (1, 2, 0))

    """
    print(np.histogram(images))
    print(np.histogram(labels))
    print(np.histogram(output_image))

    print(images.shape)
    print(labels.shape)
    """
    print(np.max(output_image))

    save_dir_triplet = args.save_dir + '/'
    os.makedirs(save_dir_triplet, exist_ok=True)

    image_file_name = args.save_dir + '/' + 'image.jpg'
    pred_file_name = args.save_dir + '/' + 'prediction.png'

    imwrite(image_file_name, image)
    imwrite(pred_file_name, output_image)

    print("saved image/prediction pair")

    if (args.visualize):
        imshow('predicted image', output_image)

    

if __name__ == '__main__':
    parser = ArgumentParser()

    parser.add_argument('--loadDir',default="/kaggle/input/erfnet-trial-model-2")
    parser.add_argument('--loadWeights', default="checkpoint.pth-3.tar")
    
    parser.add_argument('--imagedir', default='/kaggle/input/dashcam/dashcam_test.webp')
    parser.add_argument('--cpu', action='store_true', default = False)

    parser.add_argument('--visualize', action='store_true', default = False)
    parser.add_argument('--save_dir',default='/kaggle/working/save')
    main(parser.parse_args(args=[]))