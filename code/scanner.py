#! /usr/bin/env python


#     License:BSD
#     This file scanner.py is to perform scanning from fixed    locations, when this node is launched, then the locations should be passed via rosservice command in another terminal, the "number" of positions and the "coordinate" should be passed through terminal to the node, then the scanning process at each location will be repeated untils the end of scanning
#
#    Maintainer: Alexander.Kang
#
#    Email: alexander.kang@tum.de
#
#    Date: 11.02.2018

import rospy
import argparse
import time
import datetime
import math
import threading
import sweep_constants
import scan_settings
import scan_exporter
import scan_utils
import scanner_base
import tf
import std_msgs.msg
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
#import point_cloud_message_creator
from scanner_output import output_json_message
from sweeppy import Sweep
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point 
from sensor_msgs.msg import PointCloud2, PointField

from scanner_3d.srv import *
 
points_counter = 0

epochs_min = 1      #scanning times
epochs_max = 10
epochs = 0

Pose_x = 0.0
Pose_y = 0.0
Pose_z = 0.0 #offset

last_x = 0.0
last_y = 0.0
last_z = 0.0

HEADER = Header(frame_id = "map")

FIELDS = [
    # coordinate(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # signal strength
    PointField(name='Intensity', offset=9, datatype=PointField.UINT8, count=1),
]

Cloud_Points = []

def handle_pose_num_request(req):
    rospy.loginfo('PoseNumberRequest Received -epochs:%s!', req.Pose_num)
    if epochs_min <= req.Pose_num <= epochs_max:
       success = True
       global epochs
       epochs =  req.Pose_num
    else:
       success = False
    return Pose_numResponse(success)

def handle_scanning_poses_request(req):
    rospy.loginfo('PoseSettingRequest Received array length %s', len(req.path_x))
    if len(req.path_x) == epochs:
       success = True

       for i in range(0, epochs):
          print "epochs = %d" %(i+1)
          global Pose_x
          Pose_x = req.path_x[i]
          global Pose_y
          Pose_y = req.path_y[i]
          global Pose_z
          Pose_z = req.path_z[i]
          print("pose x = %f") %Pose_x
          print("pose y = %f") %Pose_y
          print("pose z = %f") %Pose_z
          # Create an exporter
          exporter = scan_exporter.ScanExporter()

          # Create a scanner base
          base = scanner_base.ScannerBase()

          #settings for sweep
          settings = scan_settings.ScanSettings()
          # Create sweep sensor, and perform scan
          with Sweep('/dev/ttyUSB0') as sweep:
               # Create a scanner object
               time.sleep(1.0)
             
               scanner = Scanner( \
               device=sweep, base=base, settings=settings, exporter=exporter)

               # Setup the scanner
               scanner.setup()

               # Perform the scan
               scanner.perform_scan()

               global epochs
               epochs -= 1    

               # Stop the scanner         
               time.sleep(1.0)
               #scanner.idle()
               base.turn_off_motors()

    else:
       success = False
    rate = rospy.Rate(0.5)
    cloud_publisher = rospy.Publisher('/sweep_node/cloudpoint', PointCloud2, queue_size = 1)   
    while not rospy.is_shutdown():
         point_cloud2 = pcl2.create_cloud(HEADER, FIELDS, Cloud_Points)       #scanner.Points 
         #publish the point cloud message via ros
         cloud_publisher.publish(point_cloud2)
         rate.sleep()
    return Set_Scanning_PoseResponse(success)

class Scanner(object):
    """The 3d scanner.
    Attributes:
        base: the rotating base
        device: the sweep scanning LiDAR
        settings: the scan settings
        exporter: the scan exporter
    """

    def __init__(self, device=None, base=None, settings=None, exporter=None):
        """Return a Scanner object
        :param base:  the scanner base
        :param device: the sweep device
        :param settings: the scan settings
        :param exporter: the scan exporter
        """
        if device is None:
            self.shutdown()
        if base is None:
            self.shutdown()
        if settings is None:
            settings = scan_settings.ScanSettings()
        if exporter is None:
            exporter = scan_exporter.ScanExporter()

        self.base = base
        self.device = device
        self.settings = settings
        self.exporter = exporter
        self.received_scan = False
        self.header = Header()

    def setup_base(self):
        """Setup the base"""
        output_json_message(
            {'type': "update", 'status': "setup", 'msg': "Resetting base to home position."})
        self.base.reset()

    def setup_device(self):
        """Setup the device"""
        reset_max_duration = 11.0

        output_json_message({'type': "update", 'status': "setup",
                             'msg': "Resetting device.", 'duration': reset_max_duration})

        # Reset the device
        self.device.reset()

        # sleep for at least the minimum time required to reset the device
        time.sleep(reset_max_duration)

        output_json_message(
            {'type': "update", 'status': "setup", 'msg': "Adjusting device settings."})

        # Set the sample rate
        self.device.set_sample_rate(self.settings.get_sample_rate())

        # Set the motor speed
        self.device.set_motor_speed(self.settings.get_motor_speed())

    def setup(self):
        """Setup the scanner according to the scan settings"""
        # setup the device, wait for it to calibrate
        self.setup_device()

        # wait until the device is ready, so as not to disrupt the calibration
        while True:
            if self.device.get_motor_ready() is True:
                break

            # Convey that the motor speed is still adjusting
            output_json_message({'type': "update", 'status': "setup",
                                 'msg': "Waiting for calibration routine and motor speed to stabilize."})

            time.sleep(0.5)

        # setup the base
        self.setup_base()

    def perform_scan(self):
        """Performs a 3d scan"""
        # Calcualte some intermediate values
        num_sweeps, angle_between_sweeps, steps_per_move = self.calculate_scan_variables()

        # Report that the scan is initiating, and start scanning
        self.report_scan_initiated(num_sweeps)
        self.device.start_scanning()
        #if not CCW:
        angle_between_sweeps = -angle_between_sweeps
        # put a 3 second timeout on the get_scans() method in case it hangs
        time_out_thread = threading.Timer(3, self.check_get_scan_timeout)
        time_out_thread.start()

        valid_scan_index = 0
        rotated_already = False

        # get_scans is coroutine-based generator returning scans ad infinitum
        for scan_count, scan in enumerate(self.device.get_scans()):
            # note the arrival time
            scan_arrival_time = time.time()
            HEADER.stamp = rospy.Time.now()
            # note that a scan was received (used to avoid the timeout)
            self.received_scan = True

            # remove readings from unreliable distances
            scan_utils.remove_distance_extremes(
                scan, self.settings.get_min_range_val(), self.settings.get_max_range_val())

            # Remove readings from the deadzone
            scan_utils.remove_angular_window(
                scan, self.settings.get_deadzone(), 360 - self.settings.get_deadzone())

            if valid_scan_index >= num_sweeps - 2:
                # Avoid redundant data in last few partially overlapping scans
                scan_utils.remove_angular_window(
                    scan, self.settings.get_deadzone(), 361)

            # Catch scans that contain unordered samples and discard them
            # (this may indicate problem reading sync byte)
            if scan_utils.contains_unordered_samples(scan):
                continue

            # Edge case (discard 1st scan without base movement and move base)
            if not rotated_already:
                # Wait for the device to reach the threshold angle for movement
                self.wait_until_deadzone(scan_arrival_time)

                # Move the base and start again
                self.base.move_steps(steps_per_move)
                rotated_already = True
                continue

            # Base angle before base rotation
            base_angle_1 = valid_scan_index * angle_between_sweeps
            # Base angle after base rotation
            base_angle_2 = (valid_scan_index + 1) * angle_between_sweeps

            converted_coords = scan_utils.transform_scan(
            scan, self.settings.mount_angle, base_angle_1, base_angle_2)

            for n, sample in enumerate(scan.samples):
                world_x = converted_coords[n, 0] + Pose_x
                world_y = converted_coords[n, 1] + Pose_y
                world_z = converted_coords[n, 2] + Pose_z
                self.exporter.writer.writerow({
                    'SCAN_INDEX': valid_scan_index,
                    'X': int(round(world_x)),
                    'Y': int(round(world_y)),
                    'Z': int(round(world_z)),
                    'SIGNAL_STRENGTH': sample.signal_strength
                })
                row = []
                row.append(float(world_x)/100)
                #n = float(converted_coords[n, 0])/100.0
                row.append(float(world_y)/100)
                row.append(float(world_z)/100)
                row.append(sample.signal_strength)
                global Cloud_Points
                Cloud_Points.append(row)

            # increment the scan index
            valid_scan_index = valid_scan_index + 1
            print "length of samples is %d" %len(scan.samples)          
            # Wait for the device to reach the threshold angle for movement
            self.wait_until_deadzone(scan_arrival_time)

            # Move the base and report progress
            self.base.move_steps(steps_per_move)
            self.report_scan_progress(num_sweeps, valid_scan_index)

            # Exit after collecting the required number of 2D scans
            if valid_scan_index >= num_sweeps:
                break

        # Stop scanning and report completion
        #time.sleep(1.0)
        #self.device.stop_scanning()
        self.report_scan_complete()
    

    def idle(self):
        """Stops the device from spinning"""
        self.device.set_motor_speed(sweep_constants.MOTOR_SPEED_0_HZ)

    def calculate_scan_variables(self):
        """ Calculates and returns intermediate variables necessary to perform a scan """
        # Calcualte the # of evenly spaced 2D sweeps (base movements) required
        # to match resolutions
        num_sweeps = int(round(self.settings.get_resolution()
                               * self.settings.get_scan_range()))  #range degree

        # Caclulate the number of stepper steps covering the angular range
        num_stepper_steps = self.settings.get_scan_range() * self.base.get_steps_per_deg()

        # Calculate number of stepper steps per move (ie: between scans)
        num_stepper_steps_per_move = int(round(num_stepper_steps / num_sweeps))

        # Actual angle per move (between individual 2D scans)
        angle_between_sweeps = 1.0 * num_stepper_steps_per_move / \
            self.base.get_steps_per_deg()

        # Correct the num_sweeps...
        # Account for the accumulated difference due to rounding
        num_sweeps = math.floor(
            1.0 * self.settings.get_scan_range() / angle_between_sweeps)
        # Account for gap introduced from splitting scans
        num_sweeps = num_sweeps + 2

        return (num_sweeps, angle_between_sweeps, num_stepper_steps_per_move)
     

    def check_get_scan_timeout(self):
        """Checks if we have received a scan from getScan... if not, exit"""
        if not self.received_scan:
            self.base.turn_off_motors()
            raise ValueError("getScan() never returned... aborting")

    def wait_until_deadzone(self, t_0):
        """ Waits the however long is required to reach the deadzone
        :param t_0: The time the sweep crossed the 0 degree mark
        """
        time_until_deadzone = self.settings.get_time_to_deadzone_sec() - \
            (time.time() - t_0)
        if time_until_deadzone > 0:
            time.sleep(time_until_deadzone)

    def report_scan_initiated(self, num_sweeps):
        """ Reports that a scan has been initiated """
        output_json_message({
          'type': "update",
          'status': "scan",
          'msg': "Initiating scan...",
          'duration': num_sweeps / self.settings.get_motor_speed(),
          'remaining': num_sweeps / self.settings.get_motor_speed()
        })

    def report_scan_progress(self, num_sweeps, valid_scan_index):       
        """ Reports the progress of a scan """
        output_json_message({
          'type': "update",
          'status': "scan",
           'msg': "Scan in Progress...",
           'duration': num_sweeps / self.settings.get_motor_speed(),
           'remaining': (num_sweeps - valid_scan_index) / self.settings.get_motor_speed()
        })

    def report_scan_complete(self):
        """ Reports the completion of a scan """
        output_json_message({
            'type': "update",
            'status': "complete",
            'msg': "Finished scan!"
        })

    def shutdown(self):
        """Print message and shutdown"""
        exit()


def main():
    """Creates a 3D scanner and gather a scan"""
    #Initialize Node and handles
    rospy.init_node('sweep_node_static_scanning', anonymous=True)
    times_service = rospy.Service('/sweep_node/times/', Pose_num, handle_pose_num_request)
    times_service = rospy.Service('/sweep_node/poses/', Set_Scanning_Pose, handle_scanning_poses_request)
    rospy.spin()
if __name__ == '__main__':
    main()
