"""Jetbot_python_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Display
import math
import numpy as np
import cv2
from matplotlib import pyplot as plt

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

wheel_height = 0.0078
wheel_radius = 0.0299

max_speed = 50

r_w = 0.0299

b=0.0525

def show_images(image, processed_image_display):
    if len(image.shape)<3:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    if image is not None:
        camera_width=image.shape[0]
        camera_height=image.shape[1]
        #print(camera_width)
        ir = processed_image_display.imageNew(image.tolist(), Display.RGB, camera_width, camera_height)
        processed_image_display.imagePaste(ir, 0, 0, False)
        processed_image_display.imageDelete(ir)


def select_rgb_white_yellow(image): 
    # white color mask
    lower = np.uint8([200, 200, 200])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(image, lower, upper)
    # yellow color mask
    lower = np.uint8([160, 160,   0])
    upper = np.uint8([255, 255, 255])
    yellow_mask = cv2.inRange(image, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked = cv2.bitwise_and(image, image, mask = mask)
    return masked
    
    
def detect_edges(image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

def filter_region(image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)

    
def select_region(image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    # first, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left  = [0, rows*0.99]
    top_left     = [0, rows*0.2]
    bottom_right = [cols*0.9, rows*0.99]
    top_right    = [cols*0.9, rows*0.2] 
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)

def hough_lines(image):
    """
    `image` should be the output of a Canny transform.
    
    Returns hough lines (not the image with lines)
    """
    return cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)

def average_slope_intercept(lines):
    left_lines    = [] # (slope, intercept)
    left_weights  = [] # (length,)
    right_lines   = [] # (slope, intercept)
    right_weights = [] # (length,)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2==x1:
                continue # ignore a vertical line
            slope = (y2-y1)/(x2-x1)
            intercept = y1 - slope*x1
            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
            if slope < 0: # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    
    # add more weight to longer lines    
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane # (slope, intercept), (slope, intercept)

def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    
    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))


def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)
    
    y1 = image.shape[0] # bottom of the image
    y2 = y1*0.6         # slightly lower than the middle

    left_line  = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)
    
    return left_line, right_line

    
def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=5):
    # make a separate image to draw lines and combine with the original later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)
    # image1 * α + image2 * β + λ
    # image1 and image2 must be the same shape.
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)

def mean_line(lines):
    if len(lines)>0:
        print(lines)
        line = np.mean(lines, axis=0, dtype=np.int32)
        line = tuple(map(tuple, line)) # make sure it's tuples not numpy array for cv2.line to work
    return line
    
#def direction_control_wheels_speed(x_old, y_old, theta_old, x_new, y_new):
    # Modify this function to generate the new wheels speed
    #d_phi_r = 10
    #d_phi_l = 10
    
    #return d_phi_r,d_phi_l

def direction_control_wheels_speed(x_old, y_old, theta_old, x_new, y_new):

    L = math.sqrt((x_new - x_old)**2 + (y_new - y_old)**2)
    #alpha = np.arctan2(y_new - y_old, x_new - x_old) - theta_old
    #L = math.sqrt((x_old - x_new)**2 + (y_old - y_new)**2)
    #L = 0.01
    y =(y_new-y_old)
    v = 0.4
    delta = (2 * abs(y))/(L ** 2)
    omega = 100.0 * delta
    right_wheel_speed = ((2*v) +(omega*b))/(2*r_w)
    left_wheel_speed = ((2*v) -(omega*b))/(2*r_w)

    return right_wheel_speed, left_wheel_speed
    
def main():
    left_motor = robot.getDevice('left_wheel_hinge')
    right_motor = robot.getDevice('right_wheel_hinge')
    
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    left_ps = robot.getDevice('left_position_sensor')
    right_ps = robot.getDevice('right_position_sensor')
    
    left_ps.enable(timestep)
    right_ps.enable(timestep)
    
    status_display = robot.getDevice('display')
    
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    height = camera.getHeight()
    width = camera.getWidth()
    
    processed_image_display = robot.getDevice('Processed')
    
    imu = robot.getDevice('imu')
    imu.enable(timestep)
    
    wheels_speed = 5.0
    
    old_ps=[0, 0]
    new_ps=[0, 0]
    
    x_n_1=-39.5
    y_n_1=40.35
    theta_n_1=0
    
    #plt.ion()
    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    
    #im = ax.imshow(np.zeros((20,20)))

    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        data = camera.getImageArray()
        image = np.array(data)
        #print(image)
        image = select_rgb_white_yellow(image)
        image_cv = cv2.rotate(image, cv2.cv2.ROTATE_90_CLOCKWISE)
        image_cv = cv2.flip(image_cv,1)
        #print(image.shape)
        #show_images(image,processed_image_display)
        gray_image = cv2.cvtColor(np.uint8(image_cv), cv2.COLOR_RGB2GRAY)
        edges_image = detect_edges(gray_image)
        #show_images(edges_image,processed_image_display)
        roi_image = select_region(edges_image)
        #im.set_data(image_cv)
        #plt.pause(0.2)
        #show_images(roi_image,processed_image_display)
        #roi_image = cv2.rotate(roi_image, cv2.cv2.ROTATE_90_CLOCKWISE)
        detected_lines = hough_lines(roi_image)
        #image = cv2.rotate(image, cv2.cv2.ROTATE_90_CLOCKWISE)
        #image = cv2.flip(image,1)
        left_line, right_line = lane_lines(image_cv, detected_lines)
        #print(right_line)
        if right_line==None:
            right_line = left_line
        path_line = mean_line((left_line, right_line))
        print(path_line)
        target_point = (path_line[1][0]-width/2.0, path_line[1][1]-height/2.0)
        if right_line[0][0]!=left_line[0][0]:
            pixel_meter= 2.0*(0.0522+0.0529)/(right_line[1][0] - left_line[1][0])
            target_point = (pixel_meter*target_point[0],pixel_meter*target_point[1])
            print(target_point)
        image_with_lines = draw_lane_lines(image_cv, (left_line, right_line, path_line))
        image_with_lines = cv2.flip(image_with_lines,1)
        image_with_lines = cv2.rotate(image_with_lines, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
        #print(detected_lines)
        show_images(image_with_lines,processed_image_display)
        # Read the sensors:
        left_speed = wheels_speed
        right_speed = wheels_speed
        
        new_ps[0]=left_ps.getValue()*wheel_radius
        new_ps[1]=right_ps.getValue()*wheel_radius
        
        
        #Add your code here to direct the robot to the correct direction
        
        left_wheel_vel = (new_ps[0]-old_ps[0])/(timestep*0.001)
        right_wheel_vel = (new_ps[1]-old_ps[1])/(timestep*0.001)
        
        angles = imu.getRollPitchYaw()
        
        
        #calculating X_dot, Y_dot of the robot coordinates 
        X_dot = (left_wheel_vel + right_wheel_vel)/2.0
        Y_dot = 0
        
        
        
        #Calculating the velocities for the world's coordinates 
        x_n_dot_i = math.cos(theta_n_1)*X_dot - math.sin(theta_n_1)*Y_dot
        y_n__dot_i = math.sin(theta_n_1)*X_dot + math.cos(theta_n_1)*Y_dot
        
        
        #solve the differential equations to find the robot's position 
        x_n = x_n_dot_i*timestep*0.001 + x_n_1
        y_n = y_n__dot_i*timestep*0.001 + y_n_1
        theta_n = angles[2]
        
        right_wheel_speed,left_wheel_speed = direction_control_wheels_speed(x_n, y_n, theta_n, target_point[0], target_point[1])
        
        #left_motor.setVelocity(wheels_speed)
        #right_motor.setVelocity(wheels_speed)
        
        left_motor.setVelocity(left_wheel_speed)
        right_motor.setVelocity(right_wheel_speed)
        
        print(str(x_n)+'     '+str(y_n)+'     '+str(theta_n))
        print(right_wheel_speed,left_wheel_speed)
        print(angles[2])
        print(left_wheel_vel,right_wheel_vel)
        old_ps[0] = new_ps[0]
        old_ps[1] = new_ps[1]
        
        x_n_1 = x_n
        y_n_1 = y_n
        theta_n_1 = theta_n
        
        #if left_speed==0 and right_speed==0:
        #    plt.ioff()
        #    plt.show()
        #    break
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #motor.setPosition(10.0)
        pass
        
        


# Enter here exit cleanup code.
if __name__ == '__main__':
    main()