import cv2
import numpy as np
import matplotlib.pyplot as plt
import lanedetection as ld

#Decription: The vehicle state class and the methods required to get each state element. Uses the latest image from the lane detection pipeline and the latest sensor data.

class VehicleState:
    """
    Digital representation of the vehicle state
    """

    def __init__(self, img, fit):
        """Default constructor

        param img: The birds eye view image from lane detection algorithm
        param fit: The path polynomial fit from lane detection algorithm
        """
        ###--------------------///Computer Representation Variables///--------------------###
        #----------Image Data----------#
        self.img = img
        self.img_dim = self.img.shape
        self.virtual_img = None

        #----------Fit Coefficients----------#
        self.fit = fit

        #----------Path Coordinate Variables----------#
        self.pic_pixy = None  #The path y coordinates in the picture frame in pixels
        self.pic_pixx = None  #The path x coordinates in the picture frame in pixels
        self.v_pixy = None  #The path y coordinates in the vehicle frame in pixels
        self.v_pixx = None  #The path x coordinates in the vehicle frame in pixels

        #----------Transformation Variables----------#
        #The meters per pixel ratios in the x and y dimension TODO: Figure out how to determine these ratios
        self.y_p2m = 20/400
        self.x_p2m = 15/400
        
        ###-----------------------------////State Variables///-----------------------------###
        #----------Vehicle Parameters----------#
        self.wrad = None  #Wheel radius
        self.voffset = 5.5 #The length from the camera to the rear axel
        self.vlength = 5.5 #The length of the vehicle
        self.vwidth = 2.2 #The width of the vehicle

        #The x and y coordinates of the vehicle frame origin
        self.vorigin_y = (self.img_dim[0] + (self.voffset // self.y_p2m))
        self.vorigin_x = int(self.img_dim[1]) // 2

        #----------Environmental Variables----------#
        self.pathy = None #The path y coordinates in the vehicle frame in meters
        self.pathx = None #The path x coordinates in the vehicle frame in meters

        #----------Runnning Variables---------#
        #The time since program initiation
        self.t = None

        #The total distance traveled by the vehicle
        self.dist = None

        #----------Vehicle Pose----------#
        #The y position in the global coordinate frame
        self.glb_y = None

        #The x position in the global coordinate frame
        self.glb_x = None

        #The angle between the global and vehicle local angles
        self.glb_heading = None

        #----------Dynamic State Variables----------#
        #The speed of the vehicle
        self.velocity = 13

        #The acceleration of the vehicle 
        self.accel_x = None
        self.accel_y = None

        if ((self.accel_x is not None) & (self.accel_y is not None)):  #TODO: Add in microcontroller code instead
            self.accel = np.sqrt(self.accel_x**2 + self.accel_y**2)

        #----------Path Relation Variables----------#
        #The lateral displacement from the reference point of the vehicle to the lane (perpendicular to the vehicle heading)
        self.lat_disp = None

        #The lateral displacement from the path to the reference point of the vehicle (perpendicular to the path)
        self.lat_pdisp = None

        #The cross-track error of the vehicle and lane
        self.cterror = None


    #----------Transform Functions----------# 

    def gen_virt_space(self, img=None, fit=None, plot=False):
        """
        Adds two vehicle lengths of "virtual space" behind the camera and extends the path into the space according to the fit

        param img: The perspective warped image
        param fit: The polynomial fit coefficients for the path
        param plot: a boolean of whether or not plot the result
        """

        if img == None:
            img = self.img
            img_dim = self.img_dim

        if fit == None:
            fit = self.fit

        #Create a vector y points that includes two vehicle lengths of "virtal space" behind the camera
        new_length = (np.rint((2*self.vlength) / self.y_p2m) + img_dim[0]).astype(int)

        #Create an image that includes the "virtual space"
        virtual_img = np.empty((new_length, img_dim[1], 3))
        virtual_img[0:img_dim[0], 0:img_dim[1], :] = img

        #Extend the path fit into the virtual space
        pic_pixy, pic_pixx = ld.Lane.gen_poly_points(new_length, fit)
        self.pic_pixy = pic_pixy
        self.pic_pixx = pic_pixx

        #Add the vehicle as a block of green pixels
        vbottom = int(img_dim[0] + (self.voffset // self.y_p2m))
        vtop = int(vbottom - (self.vlength // self.y_p2m))
        vleft = int((img_dim[1] // 2) - (self.vwidth // self.x_p2m))
        vright = int((img_dim[1] // 2) + (self.vwidth // self.x_p2m))
        virtual_img[vtop:vbottom, vleft:vright, 1] = 255

        self.virtual_img = cv2.cvtColor(virtual_img.astype('uint8'), cv2.COLOR_BGR2RGB)

        if plot == True:
            figure, axis1 = plt.subplots(1,1)
            axis1.imshow(self.virtual_img)
            axis1.plot(pic_pixx, pic_pixy)
            axis1.set_title("Warped Image with Virtual Space")
            plt.show()

        return pic_pixy, pic_pixx

    def vframe_trans(self, y=None, x=None):
        """
        Transforms the fit from lane detection into the vehicle's frame

        param y: The path y coordinates in the image frame in pixels
        param x: The path x coordinates in the image frame in pixels
        param img_dim: The dimensions (in np.shape format) of the image that the path was extracted from
        return: The path y and x coordinates in the vehicle local frame in pixels
        """

        if (y == None) and (x == None):
            y = self.pic_pixy
            x = self.pic_pixx

        img_dim = self.img_dim

        #Transform the results to vehicle local frame
        v_pixy = -(y - self.vorigin_y)
        v_pixx = x - self.vorigin_x
        self.v_pixy = v_pixy
        self.v_pixx = v_pixx

        return v_pixy, v_pixx

    def pix2meter(self, y=None, x=None):
        """
        Transform vectors of x and y coordinates from pixels to meters

        param y: The path y coordinates in the vehicle frame in pixels
        param x: The path x coordinates in the vehicle frame in pixels
        return: The path x and y coordinates in the vehicle frame in meters
        """

        if (y == None) and (x == None):
            y = self.v_pixy
            x = self.v_pixx

        pathy = y*self.y_p2m
        pathx = x*self.x_p2m

        self.pathy = pathy
        self.pathx= pathx

        return pathy, pathx

    #----------Measured State Variable Getters----------#
    
    def get_distance(self):
        """
        Gets the total distance traveled

        return: The total distance traveled
        """
    
    def get_velocity(self):
        """
        Gets the velocity from the tachometer

        return: The velocity (as a scalar)
        """

    def get_accel(self):
        """
        Gets the acceleration from the IMU

        return: A tuple of acceleration in the format accel_y, accel_x
        """

    def get_global_heading(self):
        """
        Gets the global heading (difference between track global and vehicle local angles) from the IMU

        return: The gloabl heading
        """

#----------Derived State Variable Calculators----------#

    def calc_pdistances(self):
        """
        Calculates the distance from the path to the vehicle refrence point at each point on the path
        """

    def calc_angerror(self):
        """
        Calculates the error between the path heading at the point on the path closest to the vehicle (as measured by lines perpendicular to the path) and the vehicle heading
        """

    def calc_cterror(self):
        """
        Calculates the cross track error
        """

    def calc_lat_disp(self, pathx, pathy):
        """
        Calculate the displacement (perpendicular to the vehicle) from the vehicle refrence point to the path 

        return: The displacement from the vehicle refrence point to the path
        """

        lat_disp = pathx(np.where(pathy == 0))
        self.lat_disp = lat_disp

        return lat_disp

    def calc_lat_pdisp(self, pathx, pathy):
        """
        Calculate the displacement (perpendicular to the path) from the path to the vehicle reference point

        return: The displacement from the path to the vehicle reference point
        """


def pipeline(pwarped, fit):

    state = VehicleState(pwarped, fit)

    pic_pixy, pic_pixx = state.gen_virt_space(plot=True)

    v_pixy, v_pixx = state.vframe_trans()

    pathy, pathx = state.pix2meter()

    return state

    


       
        










