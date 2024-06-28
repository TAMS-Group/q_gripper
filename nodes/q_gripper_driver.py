#! /usr/bin/env python 
#
# q_gripper_driver.py
#
# Basic Python ROS node to control the 2-fingered "q_gripper"
# built from four Feetech SCS series servos.
#
# Receives joint_states and other 
# data from the servos and forwards user-commands to the servos.
# Also publishes a sensor_msgs/Joy message with raw sensor data,
# including temperature/voltage/current for the servos.
#
# Provides (text/telnet) style commands on service ~command,
# and accepts joint position/effort goals on topic ~joint_goal.
# In addition, basic parallel-gripper operation is triggered
# by sending a float (radians) to ~simple_goal.
#
# 2022.07.04 - copy from "qbsc_gripper.py" 
# 2021.01.17 - created (based on screwdriver.py)
#
# (c) 2021, 2022 fnh, hendrich@informatik.uni-hamburg.de
#

import collections
import socket
import serial
import math
import numpy as np
import threading
import traceback

# import gc
# gc.set_debug(gc.DEBUG_STATS)
# print( gc )

import rospy
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg 

import q_gripper.srv

rospy.init_node( 'q_gripper', anonymous=False )

# global stuff
#
verbose      = 1
have_text_command = 0
text_command = "?" # help

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

def init_joints(hand_name_='q_gripper'):
    global hand_name, n_joints, joint_names, joint_index
    global js, js_errors
    # left finger / right finger, J2 = proximal J1 = distal  (same as Shadow hand)
    #
    hand_name = hand_name_
    n_joints = 1
    short_joint_names  = ['FJ' ]  
    joint_names  = [hand_name + '/' + s for s in short_joint_names]
    joint_index = {}

    for j in range( len(short_joint_names) ):
        joint_index[ short_joint_names[j]] = j
    for j in range( len(joint_names) ):
        joint_index[ joint_names[j]] = j

    if verbose > 3:
        print( joint_index ) # debugging only


    # JointState and raw sensor Joy messages
    #
    js           = sensor_msgs.msg.JointState()  # position + velocities + torques (radians, rad/s, Nm)
    js.name      = joint_names
    js.position  = np.zeros( n_joints )
    js.velocity  = np.zeros( n_joints )
    js.effort    = np.zeros( n_joints )

    js_errors    = sensor_msgs.msg.JointState()  # current pos + current vel + pos error in one message
    js_errors.name      = joint_names;
    js_errors.position  = np.zeros( n_joints )
    js_errors.velocity  = np.zeros( n_joints )
    js_errors.effort    = np.zeros( n_joints )

init_joints()

torques_joy  = sensor_msgs.msg.Joy()         # raw servo torques (counts)
temp_joy     = sensor_msgs.msg.Joy()         # temperatures
volt_joy     = sensor_msgs.msg.Joy()         # voltages 
curr_joy     = sensor_msgs.msg.Joy()         # currents 
forces_joy   = sensor_msgs.msg.Joy()         # strain-gauge forces (raw)

previous_positions_stamp = None
previous_torques_stamp = None
previous_torques = np.zeros( n_joints )
previous_positions = np.zeros( n_joints )

# loadcell stuff
#
n_load_cells = 1
grams_to_newtons = 0.00981
load_cell_bias = np.zeros( n_load_cells )
load_cell_gains = np.ones( n_load_cells )
forces_averaged = []
for i in range( n_load_cells ):
    forces_averaged.append(collections.deque( maxlen=100 ))




# IMU names and messages
#
# imu_names = ['palm_imu', 'LF_imu', 'RF_imu' ] # palm IMU on Arduino nano 33 IOT, two IMUs on middle finger and thumb
imu_names = ['palm_imu' ] # only the main 'palm' IMU
imu_messages = []
imu_raw_joy_messages = []
imu_calibration = []
imu_averaged = {}

for i in range( len(imu_names) ):
    imu_raw_joy_messages.append( sensor_msgs.msg.Joy() )
    imu_messages.append( sensor_msgs.msg.Imu() )
    imu_calibration.append( {} )

    # default gain is 1.0, default bias is 0 counts. Scale depends on the IMU used
    # and the selected sensitivity and should be set via parameters.
    #
    imu_calibration[i][ 'accel_scale' ] = 4.0 * 9.81 / 32768.0  # 4g max at 16-bit signed, default LSM6DS3
    imu_calibration[i][ 'accel_gain_x' ] = 1.0
    imu_calibration[i][ 'accel_gain_y' ] = 1.0
    imu_calibration[i][ 'accel_gain_z' ] = 1.0
    imu_calibration[i][ 'accel_bias_x' ] = 0.0
    imu_calibration[i][ 'accel_bias_y' ] = 0.0
    imu_calibration[i][ 'accel_bias_z' ] = 0.0

    imu_calibration[i][ 'gyro_scale' ] = 250.0 * DEG2RAD / 32768  # 250 deg/s max at 16 bit signed
    imu_calibration[i][ 'gyro_gain_x' ] = 1.0
    imu_calibration[i][ 'gyro_gain_y' ] = 1.0
    imu_calibration[i][ 'gyro_gain_z' ] = 1.0
    imu_calibration[i][ 'gyro_bias_x' ] = 0.0
    imu_calibration[i][ 'gyro_bias_y' ] = 0.0
    imu_calibration[i][ 'gyro_bias_z' ] = 0.0

    # running averages stuff (note: raw data)
    imu_averaged[ imu_names[i] ] = {}

    for k in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
        imu_averaged[ imu_names[i] ][ k ] = collections.deque( maxlen=100 )

# print( '>>>>>>> len(imu) ' + str( len( imu_calibration )))


# default servo calibration (including cw/ccw rotation direction)
# 


joint_offsets = [ rospy.get_param( '~joint_offset' ) ]
joint_scale   = [ 1024.0/90.0] # degrees
vel_scale     = [ 1.0/4096 ]  # SMS servo 4096 steps/2pi SCS 1024 steps/2pi
servo_signs   = [ -1]

mutex = threading.Lock()
joint_state_goal= sensor_msgs.msg.JointState()
have_new_motion_goal = False


def getp( j, counts ):  # joint_index and raw position counts 
     value = DEG2RAD*( (counts - joint_offsets[j]) * servo_signs[j] / joint_scale[j])
     return value


def getv( j, counts ):
     value = DEG2RAD * counts * vel_scale[j]
     return value


def radians_to_counts( j, radians ): # map ROS angle (rad) to SCS servo #j counts
     value = int( joint_offsets[j] + (radians * RAD2DEG)*servo_signs[j]*joint_scale[j] )
     # print( "radians_to_counts: joint " + str(j) + " radians " + str(radians) + " -> " + str( value ))
     return value


#
# perform a tara (=reset-bias) operation for the requested sensors,
# where loadcells = 0 1 2 3 ... and IMU channels are selected by ax..gz.
# For example "tara 023 palm_imu ax gy gz" will perform tara on the
# loadcells 0, 2, and 3, and the accelerometer ax and gyroscope gy gz
# channels of the "palm_imu".
#
def tara( msg ):
    global load_cell_bias, load_cell_gains, forces_averaged
    global imu_calibration, imu_averaged

    cmd = msg
    if msg.startswith( 'tara'): 
        cmd = msg[4:] # no substring method, need slicing
 
    print( "... tara command: >" + str(cmd) + "< ..." )

    mutex.acquire()

    # load-cell tara selected by channel IDs '0'..'n':
    #
    for i in range( len( load_cell_gains) ):
        if cmd.find( str(i) ) >= 0:
            if verbose > 3:
                print( "### tara for load cell index " + str(i) + "..." )

            mean_force = 1.0 * sum( forces_averaged[i] ) / len( forces_averaged[i] )
            load_cell_bias[i] = -load_cell_gains[i]*mean_force

            if verbose > 3:
                print( "### tara for load cell " + str(i) 
                     + " n_samples " + str(len( forces_averaged[i] ))
                     + " sum " + str( sum( forces_averaged[i] ))
                     + " force_ave " + str( mean_force )
                     + " new bias " + str( load_cell_bias[i] ))
            else:
                print( "... load-cell[" + str(i) + "] new bias:" + str( load_cell_bias[i] ))
    print( "... load-cell tara ok." )

    # gyro tara selected by 'g' (all IMUs), calculate running average from current data
    #
    gyro_channel_keys = ['gx', 'gy', 'gz']
    gyro_bias_names   = ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z'] 
    for i in range( len( imu_names )):
        imu_name = imu_names[i]

        for k in range(len(gyro_channel_keys)):
            if cmd.find( gyro_channel_keys[k] ) < 0: 
                continue    # skip gx gy gz if not selected in command
            print( "### Found " + str(gyro_channel_keys[k]) + " in command: " + str(cmd) )

            n_samples = len( imu_averaged[imu_name][gyro_channel_keys[k]] )
            print( "### have " + str(n_samples) + " samples from IMU, averaging now..." )

            if n_samples <= 0:
                continue    # also skip if not enough values

            ss = sum( imu_averaged[imu_name][gyro_channel_keys[k]] )
            bb = -ss / n_samples
            print( "### " + imu_name + ": new gyro bias " + str(k) + ": " + str( bb ))

            imu_calibration[i][gyro_bias_names[k]] = bb

    print( "... gyro tara ok." )

    # accel tara selected by 'a' (all IMUs)
    #
    if cmd.find( "a" ) >= 0:
        print( "... accel tara requested, but not implemented yet!" )

    mutex.release()
    print( "... tara ok." )
    return



def handle_text_command( request ):
    global text_command
    global have_text_command
    global verbose 
    print( "####################" )
    print( "handle_text_command: Received '" + str( request.command ))

    if (have_text_command != 0):
       print( "handle_text_command: Busy; command ignored!" )
       return False

    # special handling for some commands...
    # 
    if (request.command.startswith( "verbose" )): # handled by us, use "V..." to set Arduino level
       try:
           verbose = int( request.command[7:] )
           print( "### new verbose level: " + verbose )
       except Exception as e:
           print( "### invalid int format for verbose command '" + request.command + "', ignored." )
      
    elif (request.command.startswith( "tara" )):
       tara( request.command )
      
    else: # forward to Arduino on next iteration 
       text_command = request.command
       have_text_command = 1

    res = q_gripper.srv.TextCommandResponse()
    res.status = "ok."
    print( "####################" )
    return res


def simple_goal_callback( msg ):
    global have_new_motion_goal
    global joint_state_goal
    # rospy.loginfo( rospy.get_called_id() + " Got new simple joint goal " + str(msg) )
    if verbose > 2:
        print( "Got new simple goal " + str(msg) )

    try: 
        mutex.acquire()
        if (have_new_motion_goal):
            rospy.logwarn( "Got new motion goal while previous was not processed, overwriting..." );

        # for the "simple" joint goal, we take the first joint position
        # and apply it to all four joints (with signs adjusted as necessary).
        # If position[1] is also given, that is used as the "delta" angle
        # for the distal phalanges (fingertips tilt inwards or outwards),
        # and position[2] is similarly used for proximal delta.
        # The rest of the message, including velocities and efforts, is ignored.
        #
        # Maybe it would be nice to take an "opening" width in meters and
        # convert to joint-angle via IK?
        # 
        angle =  msg.position[0]

        delta = 0.0
        if len(msg.position) > 1:
            delta = msg.position[1]

        tilt = 0.0
        if len(msg.position) > 2:
            tilt = msg.position[2]

        joint_state_goal.position = [angle+delta, -angle+tilt,  angle+delta, -angle+tilt]

        joint_state_goal.header.stamp = msg.header.stamp
        joint_state_goal.velocity = []
        joint_state_goal.effort = []
        have_new_motion_goal = True

    finally:
        mutex.release()
    return



def joint_goal_callback( msg ):
    global have_new_motion_goal
    global joint_state_goal
    # rospy.loginfo( rospy.get_called_id() + " Got new joint goal " + str(msg) )
    if verbose > 3:
        print( "... Got new joint goal: " + str(msg) )

    try: 
        mutex.acquire()
        if (have_new_motion_goal):
            rospy.logwarn( "Got new motion goal while previous was not processed, overwriting..." );

        # some ROS nodes use urdf order, some alphabetic, and then some.
        # Fact is, we need to (re-) order the joints here from whatever 
        # the sending node uses to our internal numbering scheme
        # 
        joint_state_goal.position = [0] * n_joints
        for i in range( len(msg.name) ):
            joint_name = msg.name[i]
            ji = joint_index[ joint_name ]
            joint_state_goal.position[ji] = msg.position[i]

            # skip velocity and effort for now

        joint_state_goal.header = msg.header
        joint_state_goal.velocity = []
        joint_state_goal.effort = []
        have_new_motion_goal = True

    finally:
        mutex.release()
    return


def get_calibrated_forces( counts ): # raw loadcell readings
    global load_cell_bias, load_cell_gains, grams_to_newtons

    N = len( load_cell_gains )
    forces = np.zeros( N )
    for i in range( N ):
        forces[i] = grams_to_newtons*(load_cell_bias[i] + counts[i]*load_cell_gains[i])
    return forces


last_seq_number = -1
last_arduino_millis = -1
n_checked_messages = 0
n_dropped_messages = 0
ave_ser_in_waiting = 0.0


def check_seq_and_stamp( seq_number, arduino_millis ):
    global last_seq_number, last_arduino_millis, n_checked_messages, n_dropped_messages
    global ave_ser_in_waiting, ser

    n_checked_messages += 1

    # initialize data on first call(s)
    #
    if last_seq_number == -1: 
        last_seq_number = seq_number
        return

    if last_arduino_millis == -1:
        last_arduino_millis = arduino_millis
        return

    # compare and warn
    # 
    seq_and_stamp_ok = True
    if (seq_number - last_seq_number) == 1: # expected case
        pass
    elif (seq_number - last_seq_number) == -255: # wrap around
        pass
    else:
        n_dropped_messages += 1
        rospy.logwarn( "### packet loss: got " + str(seq_number) + " previous " + str(last_seq_number) 
                    + "   ave in_waiting: " + str( ave_ser_in_waiting )
                    + "   lost " + str(n_dropped_messages) + " / " + str(n_checked_messages) + " received." )
        seq_and_stamp_ok = False

    if (arduino_millis - last_arduino_millis) < 10: # ok
        pass
    elif (arduino_millis + 256 - last_arduino_millis) < 10: # wrap around
        pass
    else:
        rospy.logwarn( "### packet delay: got " + str(arduino_millis) + " previous " + str(last_arduino_millis) 
                    + " (msecs.)" 
                    + "   ave in_waiting: " + str( ave_ser_in_waiting )
                    + "   lost " + str(n_dropped_messages) + " / " + str(n_checked_messages) + " received." )
        seq_and_stamp_ok = False

    if seq_and_stamp_ok and verbose > 4:
        print( "... seq_and_stamp ok: " + str( seq_number ) + "   " + str( arduino_millis ))

    # check and update pyserial/kernel input buffer
    ave_ser_in_waiting = 0.99*ave_ser_in_waiting + 0.01*ser.in_waiting
 
    # update
    # 
    last_seq_number = seq_number
    last_arduino_millis = arduino_millis
    return



def q_gripper_driver():
    global verbose, js
    global have_text_command, text_command
    global have_new_motion_goal, joint_state_goal
    global previous_positions, previous_positions_stamp
    global previous_torques, previous_torques_stamp
    global ser, ave_ser_in_waiting
    global hand_name, joint_index, joint_names, n_joints

    global n_sensors, forces_bias, forces_gains
    global set_zero_bias_in_progress, set_zero_bias_start_time, set_zero_bias_interval, set_zero_bias_channels
    global set_gains_in_progress, set_gains_tokens
    print( "q_gripper driver node init...\n" )

    # UDP Socket
    # ARDUINO_IP = "192.168.104.97";   # unused, hardcoded in INFFUL network setup
    # UDP_IP = "134.100.13.101"        # our own IP address
    # UDP_PORT = 11333;                # the port we use, must match Arduino settings
    # 
    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    # sock.bind((UDP_IP, UDP_PORT))
    # print( "got the UDP socket..." )

    # ROS node stuff. To avoid python path trouble, the driver script 
    # is called q_gripper_driver.py, but the node and the topics
    # are called q_gripper/...
    #

    if rospy.has_param( '~verbose' ):
        verbose = rospy.get_param( '~verbose' )

    if rospy.has_param( '~hand_name' ):
        init_joints(rospy.get_param( '~hand_name' ))

    rate_hz = 500.0; 
    if rospy.has_param( '~rate' ):
        rate_hz = rospy.get_param( '~rate' )
    rate = rospy.Rate( rate_hz )
    print( '... loop rate is ' + str(rate_hz) + ' hz.' )
    # rospy.logerr( '... loop rate is ' + str(rate_hz) + ' hz.' )


    # Serial-port stuff 
    # 
    # port_name = '/dev/ttyUSB0'  # Arduino nano, ESP32 NodeMCU, ...
    port_name = '/dev/ttyACM1'  # Teensy, Nano 33 IOT, Feather Wifi ...  
  
    if rospy.has_param( '~port_name' ):
        port_name = rospy.get_param( '~port_name' )
        print( '... serial port name is ' + port_name )

    baudrate = 115200
    if rospy.has_param( '~baudrate' ):
        baudrate = rospy.get_param( '~baudrate' )

    ser = serial.Serial( port_name, baudrate=baudrate )
    print( "... got the serial port ('" + port_name + ") baudrate '" + str(baudrate) )

    #
    # IMU calibration: ranges/scales and offsets/biases
    #
    for i in range( len( imu_names )):
        if (verbose > 1): print( '... checking calibration parameters for IMU ' + imu_names[i] )
        nnames = ['gyro_scale', 
                  'gyro_gain_x', 'gyro_gain_y', 'gyro_gain_z', 
                  'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z', 
                  'accel_scale',  
                  'accel_gain_x', 'accel_gain_y', 'accel_gain_z',
                  'accel_bias_x', 'accel_bias_y', 'accel_bias_z']
        nvals  = [ 250.0*DEG2RAD/32768.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                   4.0*9.81/32768.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 ] 

        for n in range( len( nnames )):
            pname = 'q_gripper/' + imu_names[i] + '/' + nnames[n]
            if (verbose > 1): print( '... checking param ' + pname )
            if rospy.has_param( pname ):
                value = rospy.get_param( pname )
                imu_calibration[ i ][ nnames[n] ] = value
                # print( imu_calibration[ i ][ nnames[n] ] )
            else:
                imu_calibration[ i ][ nnames[n] ] = nvals[n]

            if (verbose > 2): print( '... param ' + pname + ' = ' + str(imu_calibration[i][nnames[n]]) )


    # ROS publishers and subscribers
    # 
    command_server = rospy.Service( '~command', q_gripper.srv.TextCommand, handle_text_command )
    print( "... created the text-command server ..." )

    iteration = 0
    running = 1
    joint_state_publisher = rospy.Publisher( '~joint_states', sensor_msgs.msg.JointState, queue_size=2 ) 
    joint_error_publisher = rospy.Publisher( '~joint_errors', sensor_msgs.msg.JointState, queue_size=2 ) 

    torques_publisher     = rospy.Publisher( '~torques',      sensor_msgs.msg.Joy, queue_size=2 )
    temperature_publisher = rospy.Publisher( '~temperatures', sensor_msgs.msg.Joy, queue_size=2 )
    voltages_publisher    = rospy.Publisher( '~voltages',     sensor_msgs.msg.Joy, queue_size=2 )
    currents_publisher    = rospy.Publisher( '~currents',     sensor_msgs.msg.Joy, queue_size=2 )
    forces_raw_publisher  = rospy.Publisher( '~forces_raw',   sensor_msgs.msg.Joy, queue_size=2 )
    forces_publisher      = rospy.Publisher( '~forces',       sensor_msgs.msg.Joy, queue_size=2 )

    # IMU publishers (raw data as Joy message, imu data as Imu message, no quaternion fusion yet)
    # 
    imu_publishers = []
    imu_raw_joy_publishers = []
    for i in range( len( imu_names )):
        imu_raw_joy_publishers.append(
            rospy.Publisher( '~' + imu_names[i] + '/imu_joy', sensor_msgs.msg.Joy, queue_size=2 ))
        imu_publishers.append(
            rospy.Publisher( '~' + imu_names[i] + '/imu_raw', sensor_msgs.msg.Imu, queue_size=2 ))

    # goal subscribers
    # 
    joint_goal_subscriber = rospy.Subscriber( '~joint_goals', sensor_msgs.msg.JointState, joint_goal_callback )
    simple_goal_subscriber = rospy.Subscriber( '~simple_goal', sensor_msgs.msg.JointState, simple_goal_callback )
    print( "... got the ROS node and publishers..." )

    # start ROS main loop
    # 
    while running and not rospy.is_shutdown():
        iteration = iteration + 1
        if (verbose > 3): 
            print( "... q_gripper loop iteration " + str(iteration) + "..." )

        try:
            if (have_text_command == 1):
                n_written = ser.write( str.encode( text_command + '\n' ) )
                ser.flush()
                have_text_command = 0
                if (verbose > 2):
                     print( "### wrote command '" + str(text_command) + "'..." )

            elif (have_new_motion_goal):
                if verbose > 4: print( "### have_new_motion_goal..." )
                try:
                    mutex.acquire()
                    # extract new motion goal and send via serial port TODO IMPLEMENT
                    # joint_state_goal
                    j0 = radians_to_counts( 0, joint_state_goal.position[0] )
                    ser.write( str.encode( 'G ' + str(j0) + ' \n' ) )
                    ser.flush()
                    have_new_motion_goal = False

                finally:
                    mutex.release()

            data = ""
            rawdata = ser.readline()
            try: 
                data = rawdata.decode( "utf-8" ) ## stupid python3 
            except Exception:
                print(traceback.format_exc())

            if (verbose > 2) and ((iteration % 1000) == 999):
                print( "average ser.in_waiting: " + str(ave_ser_in_waiting))

            if (verbose > 5) or (verbose > 3) and ((iteration % 10000) == 9999):
                print( "received message: " + str(data))

            if (data[0] == 'D'): # old style joint_state packet

                (str_D, seq_number, stamp, str_J, p1, str_V, v1, str_E, e1) = \
                [t(s) for t,s in zip(( str, int, int, 
                                      str, int, 
                                      str, int, 
                                      str, int ), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                js.header.frame_id = 'q_gripper'
                js.header.stamp    = rospy.Time.now()
                js.name            = joint_names
                js.position        = [getp(0,p1)]
                js.velocity        = [getv(0,v1)]
                js.effort          = [e1]

                joint_state_publisher.publish( js )

            elif (data[0] == 'I'): # servo currents
                (str_I, seq_number, stamp, str_amps, u1,) = \
                [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                curr_joy.header.frame_id = 'q_gripper/servo_currents'
                curr_joy.header.stamp    = rospy.Time.now()
                curr_joy.axes            = []
                curr_joy.axes.append( u1 )
                currents_publisher.publish( curr_joy )

            elif (data[0] == 'E'): # servo position errors: 'b'E 102 166 pos 0 0 0 0\n''
                (str_E, seq_number, stamp, str_pos, err1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                # joint_position_errors packet: abuse JointState so that we has pos,vel,pos-error in one message
                # FIXME: this currently uses/publishes RAW values (scservo counts), but SHOULD publish
                # SI units (rad, rad/s, delta rad)
                #
                js_errors.header.frame_id = 'q_gripper/joint_pos_joint_goals_pos_errors'
                js_errors.header.stamp    = rospy.Time.now()
                js_errors.name            = joint_names
                js_errors.position        = [err1]
                for i in range( n_joints):
                    try:
                        js_errors.velocity[i] = (joint_state_goal.velocity[i] - js.velocity[i])
                        js_errors.effort[i]   = (joint_state_goal.effort[i] - js.effort[i])
                    except:
                        js_errors.velocity[i] = math.nan
                        js_errors.effort[i] = math.nan

                joint_error_publisher.publish( js_errors )

            elif (data[0] == 'F'): # load-cell forces
                (str_F, seq_number, stamp, str_forces, f1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                # save raw values for use in tara() operation
                load_cell_counts = [f1]
                for i in range( n_load_cells ):
                    forces_averaged[i].append( load_cell_counts[i] )

                forces_joy.header.frame_id = 'q_gripper/loadcell_forces_raw'
                forces_joy.header.stamp    = rospy.Time.now()
                forces_joy.axes            = get_calibrated_forces( load_cell_counts )
                forces_raw_publisher.publish( forces_joy )

            elif (data[0] == 'G'): # IMU: 'b'G 88 173 0 accel -5 -37 8235 gyro 13 -47 -59 \n''
                (str_I, seq_number, stamp, imu_index, str_accel, ax, ay, az, str_gyro, gx, gy, gz ) = \
                  [t(s) for t,s in zip(( str, int, int, int, str, int, int, int, str, int, int, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                joy = imu_raw_joy_messages[ imu_index ]
                joy.header.frame_id = hand_name + '/' + imu_names[ imu_index ] + '/frame'
                joy.header.stamp    = rospy.Time.now()
                joy.axes            = []

                joy.axes.append( ax )
                joy.axes.append( ay )
                joy.axes.append( az )
                joy.axes.append( gx )
                joy.axes.append( gy )
                joy.axes.append( gz )

                imu_raw_joy_publishers[imu_index].publish( joy )

                # put into ring-buffers for averaging/tara
                # 
                imu_counts = [ax, ay, az, gx, gy, gz]
                channels = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
                for k in range( 6 ):
                    # print( "### " + str(k) + " counts " + str( imu_counts[k] ))
                    # print( "    " + str( channels[k] ))
                    # print( "<<< " + str(imu_averaged[imu_names[imu_index]][channels[k]]) )
                    imu_averaged[ imu_names[imu_index] ][channels[k]].append( imu_counts[k] )

                # FIXME: replace hardcoded values for LSM6DS3 and MPU6050 with params
                #
                # accel_scale = 4.0 * 9.81 / 32768.0   #  4g max at 16-bit signed, default for LSM6DS3
                # gyro_scale  = 250.0 * DEG2RAD / 32768  # 250 deg/s max at 16 bit signed

                CAL = imu_calibration[ imu_index ]

                imu_msg = imu_messages[ imu_index ]
                imu_msg.header.frame_id = hand_name + '/' + imu_names[ imu_index ] + '/frame'
                imu_msg.header.stamp    = joy.header.stamp
 
                # multiply counts (ax,ay,az) with channel gains, add bias, convert to SI units scale
                #
                aa = CAL[ 'accel_scale' ]
                imu_msg.linear_acceleration.x = aa * (CAL[ 'accel_bias_x'] + ax * CAL[ 'accel_gain_x' ] )
                imu_msg.linear_acceleration.y = aa * (CAL[ 'accel_bias_y'] + ay * CAL[ 'accel_gain_y' ] )
                imu_msg.linear_acceleration.z = aa * (CAL[ 'accel_bias_z'] + az * CAL[ 'accel_gain_z' ] )

                gg = CAL[ 'gyro_scale' ]
                imu_msg.angular_velocity.x = gg * (CAL[ 'gyro_bias_x' ] + gx * CAL[ 'gyro_gain_x' ] )
                imu_msg.angular_velocity.y = gg * (CAL[ 'gyro_bias_y' ] + gy * CAL[ 'gyro_gain_y' ] )
                imu_msg.angular_velocity.z = gg * (CAL[ 'gyro_bias_z' ] + gz * CAL[ 'gyro_gain_z' ] )
                imu_publishers[ imu_index ].publish( imu_msg )

            elif (data[0] == 'M'): # servo torques (_m_oments): 'b'M 93 168 trq -1 -1 -1 -1\n''
                (str_M, seq_number, stamp, str_trq, u1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                torques_joy.header.frame_id = 'q_gripper/servo_torques'
                torques_joy.header.stamp    = rospy.Time.now()
                torques_joy.axes            = []
                torques_joy.axes.append( u1 )
                torques_publisher.publish( torques_joy )


            elif (data[0] == 'P'): # servo positions: 'b'P 92 167 pos -1 -1 -1 -1\n''
                 # servo/joint positions packet (raw counts),
                 # velocities calculated from numerical difference,
                 # efforts used from last torque packet

                (str_P, seq_number, stamp, str_pos, p1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                js.header.frame_id = 'q_gripper' # 'q_gripper'
                js.header.stamp    = rospy.Time.now()
                js.name            = joint_names
                current_positions  = [getp(0,p1)]

                js.position        = current_positions
                js.effort          = previous_torques

                # js.velocity        = [getv(0,v1), getv(1,v2), getv(2,v3), getv(3,v4)]
                # 
                if previous_positions_stamp is not None:
                    delta_t            = (js.header.stamp - previous_positions_stamp).to_sec()  # now - previous time
                    delta_millis       = (stamp - js_millis) / 1000.0
                    if (delta_millis < 0): delta_millis = delta_millis + 0.256  # millis() & 0xff
                    if verbose > 4:
                        print( "... delta_t pos: ROS " + str( delta_t ) + "  Arduino " + str( delta_millis ) + " raw diff " + str((stamp - js_millis)/1000.0) );

                    if verbose > 2 and abs(delta_t - delta_millis) > 0.02:
                        rospy.logerr( "... delta_t: ROS " + str( delta_t ) + "  Arduino " + str( delta_millis ));

                    # js.velocity        = (np.array(current_positions) - np.array(previous_positions)) / delta_tdd
                    js.velocity        = (np.array(current_positions) - np.array(previous_positions)) / delta_millis
                else:
                    js.velocity        = np.zeros( n_joints )

                previous_positions = current_positions
                previous_positions_stamp = js.header.stamp
                js_millis = stamp # arduino millis
                joint_state_publisher.publish( js )

            elif (data[0] == 'T'): # motor temperatures
                (str_T, seq_number, stamp, str_t, t1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                temp_joy.header.frame_id = 'q_gripper/servo_temperatures'
                temp_joy.header.stamp    = rospy.Time.now()
                temp_joy.axes            = []
                temp_joy.axes.append( t1 )
                temperature_publisher.publish( temp_joy )

            elif (data[0] == 'U'): # servo voltages
                (str_U, seq_number, stamp, str_volts, u1) = \
                  [t(s) for t,s in zip(( str, int, int, str, int), data.split()) ]

                check_seq_and_stamp( seq_number, stamp )

                volt_joy.header.frame_id = 'q_gripper/servo_voltages'
                volt_joy.header.stamp    = rospy.Time.now()
                volt_joy.axes            = []
                volt_joy.axes.append( u1 )
                voltages_publisher.publish( volt_joy )

            else: 
                if data == "'\r\n":
                    pass
                elif data.startswith( "..." ):
                    if verbose > 2: print( data )
                else:
                    print( "Unknown data/packet ignored: >" + str(data) + "< len: " + str(len(data)) )
                    if verbose > 4:
                        for j in range( len( data )):
                            print( ord(data[j]) )

            # rate.sleep() # drops serial data as the buffer fills up too quickly...
            # So, check buffer fill level and only wait if capacity remains
            if ser.in_waiting < 200:
                rospy.sleep( rospy.Duration( 0.002 ))
            elif ser.in_waiting < 500:
                rospy.sleep( rospy.Duration( 0.001 ))


        except ValueError:
            print( "Parsing data failed: '" + str(data) + "'" )
            continue

        except KeyboardInterrupt:
            running = False
            pass

    ser.close()
    print( "q_gripper node driver stopped." )
    


if __name__ == '__main__':
    try:
        q_gripper_driver()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass

