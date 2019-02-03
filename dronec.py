#!/usr/bin/env python
"""
Autonomous drone Mission control.
AUTHOR: Bashir Hassan
"""
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import math
import pymavlink
from pymavlink import mavutil
import getch
import sys
def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if vehicle.armed and vehicle.location.global_relative_frame.alt > 2:
        print "\n\tVehicle armed and possible flying, aborting take off!\n"
        return
    print "Basic pre-arm checks"
    # Don't let the user try to fly autopilot is booting
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
        
    print "Arming motors"
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print "Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    except KeyboardInterrupt:
        print "Keyboard Interrupt on takeoff."
        print "\n\nLanding!\n\n"
        vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        print 'The drone has landed!'
        pass

def shows_data(vehicle):
    """
    show vehicle data.
    """
    print "Get some vehicle attribute values:"
    print "attitude: %s" % vehicle.attitude 
    print "velocity: %s" % vehicle.velocity 
    print "channels: %s" % vehicle.channels
    print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
    print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
    print "GPS: %s" % vehicle.gps_0
    print "Battery: %s" % vehicle.battery
    print "Last Heartbeat: %s" % vehicle.last_heartbeat
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "armed: %s" % vehicle.armed
    print "Mode: %s" % vehicle.mode.name
    print "groundspeed: %s" % vehicle.groundspeed
    print "airspeed: %s" % vehicle.airspeed
    time.sleep(0.01)

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)

def diamond(vehicle):
    """
    Move vehicle in direction based on diamond points.
    """
    NORTH=2
    SOUTH=-2
    EAST=2
    WEST=-2
    UP=-0.5
    DOWN=0.5
    DURATION=10
    # Shape shape
    print "Making a diamond!"
    condition_yaw(vehicle,0)
    send_ned_velocity(vehicle,NORTH,0,0,DURATION)
    print "Flying for 20 seconds direction NORTH!"
    #send_ned_velocity(vehicle,0,0,0,5)
    condition_yaw(vehicle,90)
    send_ned_velocity(vehicle,0,EAST,0,DURATION)
    print "Flying for 20 seconds direction EAST!"
    #send_ned_velocity(vehicle,0,0,0,5)
    condition_yaw(vehicle,180)
    send_ned_velocity(vehicle,SOUTH,0,0,DURATION)
    print "Flying for 20 seconds direction SOUTH!"
    #send_ned_velocity(vehicle,0,0,0,5)
    condition_yaw(vehicle,270)
    send_ned_velocity(vehicle,0,WEST,0,DURATION)
    print "Flying for 20 seconds direction WEST!"
    #send_ned_velocity(vehicle,0,0,0,5)
    print("Going North, East and up")
    condition_yaw(vehicle,90)
    send_ned_velocity(vehicle,NORTH,EAST,UP,DURATION)
    print("Going South, East and down")
    condition_yaw(vehicle,90)
    send_ned_velocity(vehicle,SOUTH,EAST,DOWN,DURATION)
    print("Going South and West")
    condition_yaw(vehicle,90)
    send_ned_velocity(vehicle,SOUTH,WEST,0,DURATION)
    print("Going North and West")
    condition_yaw(vehicle,90)
    send_ned_velocity(vehicle,NORTH,WEST,0,DURATION)
    print "Returning to Launch"
    vehicle.mode    = VehicleMode("RTL")
    print "Waiting 10 seconds RTL"
    time.sleep(10)
    print "Landing the Aircraft"
    vehicle.mode    = VehicleMode("LAND")

def build_loop_mission(vehicle, loop_center, loop_radius, altitude):
    """
    Adds a takeoff command and 12 waypoint commands to the current mission.
    The waypoints are positioned to form a dodecagon with vertices at loop_radius around the specified
    LocationGlobal (loop_center).
    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    Modified from Dronekit-python
    """
    cmds = vehicle.commands
    print " Clearing any existing commands"
    cmds.clear()
    print " Building loop waypoints."
    # Add new commands. The meaning/order of the parameters is documented in the Command class.
    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, 10))
    # Define the twelve MAV_CMD_NAV_WAYPOINT locations and add the commands
    for n in range(0, 11, 1):
        d_north = math.sin(math.radians(n*30))*loop_radius
        d_east = math.cos(math.radians(n*30))*loop_radius
        point = get_location_metres(loop_center, d_north, d_east)
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, point.lat, point.lon, altitude))
    print " Upload new commands to vehicle"
    cmds.upload()
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
       0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()
    download_mission(vehicle)

def adds_square_mission(vehicle,aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """ 
    cmds = vehicle.commands
    print " Clear any existing commands"
    cmds.clear() 
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    
    print " Upload new commands to vehicle"
    cmds.upload()

#########################################
#
# CONNECTION
#
#########################################
# Connection to the vehicle
print "Connecting to drone...."
vehicle = connect('/dev/ttyAMA0', baud = 57600,wait_ready=False)
print "Connected..."
clear_mission(vehicle)
print "Mission	cleared..."
print "\n\nMission start...!!\n\n"
while True:
    # wait for user input
    string = raw_input ('Enter Command: ')
    word = string.split()
    word1 = word[0]
    # take off the drone for a scpecific Altitude
    if word1 == 'takeoff':
    	arm_and_takeoff(vehicle,2)
    	time.sleep(10)
    	print "\n\nReturning to Launch!\n\n"
        vehicle.mode = VehicleMode("RTL")
        time.sleep(15)
        print 'The drone has returned!'
        time.sleep(1)
               
    elif word1 == 'fly':
        if word[1] == 'auto':
                arm_and_takeoff(vehicle,2)
                time.sleep(10)
                vehicle.mode = VehicleMode("AUTO")
        # fly to shape a square
        elif word[1] == 'square':
                print 'Flight control: square'
                time.sleep(1)
                size = 4
                new_commands = adds_square_mission(vehicle,vehicle.location,size)
                print "%d commands in the mission!" % new_commands
                # Make sure that mission being sent is displayed
                time.sleep(2)
                arm_and_takeoff(vehicle,2)
                print "Starting mission"
                # Set mode to AUTO to start mission
                vehicle.mode = VehicleMode("AUTO")
                vehicle.flush()
                    
            # fly to shape a diamond
        elif word[1] == 'diamond':
                print 'Flight control: diamond'
                diamond(vehicle)
                time.sleep(2)
                arm_and_takeoff(vehicle,2)
                print "Starting mission"
                # Set mode to AUTO to start mission
                vehicle.mode = VehicleMode("AUTO")
                vehicle.flush()
                    
            # fly to shape a dodecagon
        elif word[1] == 'dodecagon':
                print 'Flight control: dodecagon'
                arm_and_takeoff(vehicle,2)
                time.sleep(1)
                loopcenter = vehicle.location.global_frame
                build_loop_mission(vehicle,loopcenter,4,2)
                vehicle.mode = VehicleMode("AUTO")
                vehicle.flush()
                    
        else:
                arm_and_takeoff(vehicle,2)
                time.sleep(1)
                time.sleep(10)
                vehicle.mode = VehicleMode("AUTO")

    elif word1 == 'show':
        shows_data(vehicle)
    
    elif word1 == 'land':
        print "\n\nLanding!\n\n"
        vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        print 'The drone has landed!'
    elif word1 == 'loiter':
        print "\n\nsetting mode to loiter!\n\n"
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(1)
        print 'loiter mode!'
    elif word1 == 'auto':
        print "\n\nsetting mode to AUTO!\n\n"
        vehicle.mode = VehicleMode("AUTO")
        time.sleep(1)
        print 'AUTO mode!'
    elif word1 == 'guided':
        print "\n\nsetting mode to GUIDED!\n\n"
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        print 'GUIDED mode!'
    elif word1 == 'circle':
        print "\n\nsetting mode to CIRCLE!\n\n"
        vehicle.mode = VehicleMode("CIRCLE")
        time.sleep(1)
        print 'CIRCLE mode!'
    elif word1 == 'althold':
        print "\n\nsetting mode to ALT HOLD!\n\n"
        vehicle.mode = VehicleMode("ALT_HOLD")
        time.sleep(1)
        print 'ALT HOLD mode!'
    elif word1 == 'stabilize':
        print "\n\nsetting mode to STABILIZE!\n\n"
        vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(1)
        print 'STABILIZE mode!'
    elif word1 == 'acro':
        print "\n\nsetting mode to ACRO!\n\n"
        vehicle.mode = VehicleMode("ACRO")
        time.sleep(1)
        print 'ACRO mode!'
    elif word1 == 'sport':
        print "\n\nsetting mode to SPORT!\n\n"
        vehicle.mode = VehicleMode("SPORT")
        time.sleep(1)
        print 'SPORT mode!'
    elif word1 == 'poshold':
        print "\n\nsetting mode to POSHOLD!\n\n"
        vehicle.mode = VehicleMode("POSHOLD")
        time.sleep(1)
        print 'POSHOLD mode!'
    elif word1 == 'return':
        print "\n\nReturning to Launch!\n\n"
        vehicle.mode = VehicleMode("RTL")
        time.sleep(15)
        print 'The drone has returned!'
    else:
        print 'Wrong Input....'
        time.sleep(0.1)

print "\n\nMission complete\n\n"
vehicle.close()
