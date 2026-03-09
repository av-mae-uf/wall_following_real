import rclpy
from rclpy.node import Node

import math
import numpy as np
import time

from datetime import date, datetime

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

D2R = math.pi/180.0
R2D = 180.0/math.pi

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('calc_steering')

        self.subscription_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.subscription_scan  # prevent unused variable warning

        self.ackerman_publisher = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

        self.d1 = 1.3  # garbage values
        self.d2 = 1.6

        self.declare_parameter('theta_deg', 25.0)
        self.declare_parameter('d_LookAhead', 2.5)
        self.declare_parameter('d_desired', 1.0)
        self.declare_parameter('speed', 0.0)
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('starting_delay', 0.0)
        self.declare_parameter('save_to_file', False)
        self.declare_parameter('file_name', 'cdc_default.csv')

        self.theta_deg = self.get_parameter('theta_deg').value
        self.d_LookAhead = self.get_parameter('d_LookAhead').value
        self.d_desired = self.get_parameter('d_desired').value
        self.speed = self.get_parameter('speed').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.starting_delay = self.get_parameter('starting_delay').value
        self.save_to_file = self.get_parameter('save_to_file').value

        my_date = date.today()
        my_now = datetime.now()
        my_string = str(my_date.year) + '_' + str(my_date.month) + '_' + str(my_date.day) + '_' + str(my_now.hour)+ '_' + str(my_now.minute) + '_'
        self.file_name = my_string + self.get_parameter('file_name').value

        self.max_speed = 6.0
        if self.speed > self.max_speed:
            self.speed = self.max_speed

        self.steering_ang_rad = 0.0
        self.steering_ang_rad_last = 0.0
        
        self.max_ang_deg = 45.0 

        self.error = 0.0
        self.error_m1 = 0.0
        self.error_m2 = 0.0

        self.start_time = time.time()

        if self.save_to_file:
            self.fp = open(self.file_name, "w")
            self.fp.write(f'Kp = {self.Kp}, Ki = {self.Ki}, Kd = {self.Kd}, speed = {self.speed}\n')
            self.fp.write(f'theta_deg = {self.theta_deg}, look_ahead = {self.d_LookAhead}, d_desired = {self.d_desired}\n')
            self.fp.write('time (sec), d_wall (m), heading_error_deg, steering_ang_deg, d1, d2\n')

    def control_vehicle(self):
        # Use d1 and d2 values to get dwall and look-ahead point and heading error
        
        d_wall, delta_e = get_steering_error(self.d1, self.d2, self.theta_deg*D2R, self.d_LookAhead, self.d_desired)

        self.error_m2 = self.error_m1
        self.error_m1 = self.error
        self.error = delta_e

        wait_time =self.get_parameter('starting_delay').value
        if time.time() - self.start_time < wait_time:
            speed = 0.0
            Kp = 0.0
            Ki = 0.0
            Kd = 0.0
        else:
            speed = self.speed
            Kp = self.Kp
            Ki = self.Ki
            Kd = self.Kd

        delta_ang_rad = Kp * (self.error-self.error_m1) \
                      + Ki * (self.error) \
                      + Kd * (self.error - 2*self.error_m1 + self.error_m2)
        
        self.steering_ang_rad_last = self.steering_ang_rad
        self.steering_ang_rad = self.steering_ang_rad_last + delta_ang_rad

        if self.steering_ang_rad > 45.0*D2R:
            self.steering_ang_rad = 45.0*D2R
        if self.steering_ang_rad < -45.0*D2R:
            self.steering_ang_rad = -45.0*D2R

        # publish the Ackerman message
        out_msg = AckermannDriveStamped()
        out_msg.drive.steering_angle = self.steering_ang_rad
        out_msg.drive.speed = speed
        self.ackerman_publisher.publish(out_msg)

        #self.get_logger().info(f'd_wall = {d_wall}, heading_error_deg = {delta_e*R2D}')

        now = time.time() - self.start_time
        if self.save_to_file and now>=wait_time:
            #time (sec), d_wall (m), heading_error_deg, steering_ang_deg
            self.fp.write(f'{now}, {d_wall}, {delta_e*R2D}, {self.steering_ang_rad*R2D}, {self.d1}, {self.d2}\n')

    def scan_callback(self, msg):    
        angle_min_deg = msg.angle_min * R2D
        angle_incr_deg = msg.angle_increment * R2D
        array_pos = round((-90.0 - angle_min_deg)/ angle_incr_deg)
        theta_deg = self.get_parameter('theta_deg').value
        num_extra_steps = round(theta_deg / angle_incr_deg)

        beam_d1 = msg.ranges[array_pos]
        beam_d2 = msg.ranges[array_pos+num_extra_steps]

        if not math.isinf(beam_d1):
            self.d1 = beam_d1
        if not math.isinf(beam_d2):
            self.d2 = beam_d2

        self.control_vehicle()
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

################################################
def get_steering_error(d1, d2, theta_rad, d_LookAhead, d_desired):
    # d1, d2, d_LookAhead, and d_desired have units of m
    # theta_rad is in units of radians (dimensionless)
    
    # returns the distance to wall and the orientation error to
    # point towards the look-ahead point
    
    debug_print = False
    
    # get coordinates of points P1 and P2 in sensor coord sys
    P1 = np.array([0, -d1])
    x2 = d2*math.cos(1.5*math.pi + theta_rad) # added 270 deg to theta
    y2 = d2*math.sin(1.5*math.pi + theta_rad)
    P2 = np.array([x2, y2])
    if debug_print:
        print(f'\nP1 = {P1}\nP2 = {P2}\n')
    
    v_vec = (P2-P1)/np.linalg.norm(P2-P1)
    x_sensor = np.array([1.0, 0.0])
    y_sensor = np.array([0.0, 1.0])
    cos_phi = np.dot(x_sensor, v_vec)
    d_wall = d1 * cos_phi
    if debug_print:
        print(f'v_vec = {v_vec}\n')
        print(f'd_wall = {d_wall}\n')
    
    S_perp = np.array([-v_vec[1], v_vec[0]])
    P_LookAhead = d_LookAhead * v_vec + (d_desired - d_wall)*S_perp
    if debug_print:
        print(f'P_LookAhead = {P_LookAhead}')
    
    S_LookAhead = P_LookAhead/np.linalg.norm(P_LookAhead)
    
    delta_e = math.atan2(S_LookAhead[1], S_LookAhead[0])
    # atan2 function will bound delta_e between -pi and +pi
    return (d_wall, delta_e)
################################################
