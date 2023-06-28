#!/usr/bin/env python

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import time

# Connect to the Vehicle (in this case a simulator running DroneKit-SITL)
connection_string = 'udpin:0.0.0.0:14550'
vehicle = connect(connection_string, wait_ready=True)

r_earth=6378137.0

def get_new_pos_location(takeOff_location, dNorth, dEast,auto,count):
    
    #Function used to move the drone to a specific location relative to the current position.

    #if auto is "yes" this position is calculated automatically.
    if manual=="yes":
        
        #Coordinate offsets in radians
        dLat = dNorth/r_earth
        dLon = dEast/(r_earth*math.cos(math.pi*takeOff_location.lat/180))

        #New position in decimal degrees
        newlat = takeOff_location.lat + (dLat * 180/math.pi)
        newlon = takeOff_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,takeOff_location.alt)

    #In other case, this position is introduced manually with the respective latitude and longitude for each point.
    else:
        
        if count==1:
            
            return LocationGlobal(41.276377, 1.988363,takeOff_location.alt)

        elif count==2:

            return LocationGlobal(41.276330, 1.988457,takeOff_location.alt)

        elif count==3:

            return LocationGlobal(41.276256, 1.988401,takeOff_location.alt)

        elif count==4:

            return LocationGlobal(41.276271, 1.988307,takeOff_location.alt)
            
        


def get_distance_meters(aLocation1, aLocation2):

    #Function that returns the distance in meters from a position
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():

    #Function that returns the distance in meters to the next waypoint
    
    nextWaypoint = vehicle.commands.next
    if nextWaypoint==0:
        
        return None
    
    missionitem=vehicle.commands[nextWaypoint-1] 
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distanceToPoint = get_distance_meters(vehicle.location.global_frame, targetWaypointLocation)
    return distanceToPoint


def download_mission():

    #Function used to download the mission of the drone.
    
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() 



def add_drone_mission(aLocation, aSize):
     
    #Function which allow to up-load the desires mission to the drone.

    cmds = vehicle.commands

    print(" Cleanning any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the 3 points locations for the dron route. As we are calling the get_new_pos_location, the point will be placed with the drone initial position as reference.
    
    point1 = get_new_pos_location(aLocation, aSize, aSize,"no",1)
    point2 = get_new_pos_location(aLocation, 0, 2*aSize,"no",2)
    point3 = get_new_pos_location(aLocation, -aSize, aSize,"no",3)
    point4 = get_new_pos_location(aLocation, 0, 0,"no",4)

    #Add the commands with the points generated previously to the drone mission.
    
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))

    print(" Upload new mission to the drone")
    cmds.upload()


def aat(aTargetAltitude):
    
    #Fuction which allows to arm the vehicle and fly to the desired altitude.
    

    print("Basic pre-arm checks")
    
    while not vehicle.is_armable:
        
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    
    #To arm the vehicle it must be in GUIDED mode, later de AUTO mode will be introduced.
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        
        print(" Waiting for arming the vehicle...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    
    while True:
        
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #IF used to check if the drone is at target altitude.
            print("Reached target altitude")
            break
        time.sleep(1)

        
print('Creating a new mission')
add_drone_mission(vehicle.location.global_frame,25)

aat(30)

print("Starting mission")

vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

while True:
    nextWaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextWaypoint, distance_to_current_waypoint()))
  
    if nextWaypoint==4:  #When we reach waypoint 4 this is true and we go to starting position.
        print("Exit, going to takeoff location")
        break;
    
    time.sleep(2)

print('Return to launch')
vehicle.mode = VehicleMode("RTL")


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
