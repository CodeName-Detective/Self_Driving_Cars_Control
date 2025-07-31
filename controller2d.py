#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake
    
    @staticmethod
    def normalize_angle(angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < - np.pi:
            angle += 2 * np.pi
        
        return angle

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('integral_error_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)
        

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            throttle_output = 0
            brake_output    = 0
            
            # Implementing PID Controller
            K_p = 1.0
            K_i = 0.2
            K_d = 0.01
            
            delta_t = t - self.vars.t_previous
            
            #Proportional Value
            error_value = v_desired - v
            
            #Integral Value
            i_value = self.vars.integral_error_previous+(error_value*delta_t)
            
            # Derivative Value
            d_value = (error_value - self.vars.error_previous)/delta_t
            
            acceleration_desired = (K_p*error_value) + (K_i*i_value) + (K_d*d_value)
            
            if acceleration_desired > 0:
                #Maps Acceleration to a continuous Value between 0 to 1 ((-1,1) ->(+1) (0,2) ->(/2) (0,1) )
                throttle_output = (np.tanh(acceleration_desired) + 1) / 2
                brake_output    = 0
                
                # Optional: Rate limit Throttle increase if needed - To prevent aggressive actions
                if throttle_output - self.vars.throttle_previous > 0.1:
                    throttle_output = self.vars.throttle_previous + 0.1
                
            elif acceleration_desired < 0:
                throttle_output = 0
                
                #Maps Acceleration to a continuous Value between 0 to 1
                brake_output    = (np.tanh(-acceleration_desired) + 1) / 2
               
            else:
                throttle_output = 0
                brake_output    = 0

            ######################################################
            ######################################################
            # IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller.
            steer_output    = 0
            
            # Stanley Controller - Front Axel Reference Frame
            
            # Calculate cross track Error Assume line that connects first point and last point of the path: y = mx + c => -mx+y-c=0 => ax+by+c = 0
            slope = (waypoints[-1][1]- waypoints[0][1])/(waypoints[-1][0]- waypoints[0][0])
            
            a = -slope
            b = 1.0
            c = -(a*waypoints[0][0]+b*waypoints[0][1]) # c = -(ax+by)
            
            K_gain = 0.3
            
            
            # Heading Error with respect to the path - line connects first and last point in the trajectory
            
            ## Slope of the trajectory
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            
            ## Slope of the trajectory - current angle = heading_error
            yaw_diff_heading = yaw_path - yaw
            yaw_diff_heading = Controller2D.normalize_angle(yaw_diff_heading)
            
            # Cross-Track Error
            current_xy = np.array([x, y])
            ## Distance between Closest point and current point - min(distance between all points)
            cross_track_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))

            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            yaw_path2ct = Controller2D.normalize_angle(yaw_path - yaw_cross_track)
        
            if yaw_path2ct > 0:
                cross_track_error = abs(cross_track_error)
            else:
                cross_track_error = - abs(cross_track_error)
        
            yaw_diff_cross_track = np.arctan(K_gain * cross_track_error / (v))

            # final expected steering
            steer_expect = Controller2D.normalize_angle(yaw_diff_cross_track + yaw_diff_heading)
            
            steer_expect = min(1.22, steer_expect)
            steer_expect = max(-1.22, steer_expect)

            #update
            steer_output = steer_expect
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v
        self.vars.throttle_previous = throttle_output
        self.vars.t_previous = t
        self.vars.error_previous = error_value
        self.vars.integral_error_previous = i_value