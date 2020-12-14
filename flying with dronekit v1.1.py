
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
from detection_delivery_point import detect

################################################################################################
# Settings
################################################################################################

connection_string       = '///dev/ttyAMA0:57600'  ##In order to connect Autopilot with Raspberry Pi

##If you desire another type of connection(i.e  telemetry connection, you should write ///dev/ttyUSB0:57600
##You can find the string by searching on google as "Dronekit connection string (telemetry,raspberrypi,nvidia jetson etc...)"

MAV_MODE_AUTO   = 4 ##Mission Mode Definition


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Initial Settings
################################################################################################

print("Connecting to the vehicle...")
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode): ##Defines the mode of the vehicle, in pixhawk if you send mavlink commands with dronekit, vehicle mode becomes "Mission" automatically.
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def get_location_offset_meters(original_location, dNorth, dEast, alt):                                                                     
    ##GPS values to the metric values transformation
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt) 
    
    
def get_distance_metres(aLocation1, aLocation2):
##Gives the distance between two points in type of metric.
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
def distance_to_current_waypoint():
##Gives the distance to the next waypoint in type of metric.
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobal(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


################################################################################################
# Listeners
################################################################################################

home_position_set = False
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True


################################################################################################
# Mission Start
################################################################################################

while not home_position_set:
    print "Home position is initializing..."
    time.sleep(1)

print(" Vehicle Type: %s" % vehicle._vehicle_type)
print(" Arm Status: %s" % vehicle.armed)
print(" System Status: %s" % vehicle.system_status.state)
print(" GPS Status: %s" % vehicle.gps_0)
print(" Altitude (Relative): %s" % vehicle.location.global_relative_frame.alt)

PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

##Ground and Air Speed definitions. 

##Ground speed is the speed vehicle moves in the horizontal direction.
##Air speed is the speed vehicle moves in the vertical direction.

vehicle.groundspeed=3
vehicle.airspeed=3

##Resets the command in the autopilot card defined before.
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

flagIsSelected = False
while not flagIsSelected:
    print("1.Gazi University YDYO Garden \n")
    print("2.Gazi University GSF Garden \n")
    choice=input("Please choose the destination \n")
    if choice == 1:
        arrival_lat = 39.781821
        arrival_lon = 32.806964
        yaw_heading=270
        flagIsSelected = True
    elif choice == 2:
        arrival_lat = 39.781734
        arrival_lon = 32.808976
        yaw_heading=135
        flagIsSelected = True
    else:  
        print("Incorrect, please choose again !!")
    
    
print "Mission is starting..."      
##30 meters above command
wp = get_location_offset_meters(home, 0, 0, 30);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, yaw_heading, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

##Heading command
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, yaw_heading, 
arrival_lat, arrival_lon, wp.alt)
cmds.add(cmd)

cmds.upload()
time.sleep(2)

# Arm vehicle
print "Vehicle is arming..."
vehicle.armed = True

print "Takeoff..."
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print "Moving to destination..."
        nextwaypoint = vehicle.commands.next
    time.sleep(1)
print " Distance to the destination : %s" % distance_to_current_waypoint()

while distance_to_current_waypoint() > 0.1:	
	time.sleep(1)


##25 meters below command 
wp = get_location_offset_meters(vehicle.location.global_relative_frame, 0, 0, -25);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
time.sleep(2)
nextwaypoint = vehicle.commands.next


##Detection of the delivery point
isDetected = false
while not isDetected:
    isDetected, rel_x, rel_y = detect(0.5)
time.sleep(10)

vehicle.mode = "HOLD"
time.sleep(10)


wp = get_location_offset_meters(vehicle.location.global_relative_frame, rel_x, rel_y, wp.alt);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

wp = get_location_offset_meters(vehicle.location.global_relative_frame, 0, 0, -4.5);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

cmds.upload()
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print("Cargo is being delivered...")
        nextwaypoint = vehicle.commands.next
    time.sleep(1)


time.sleep(10)
vehicle.send_mavlink(servo_msg)

time.sleep(10)
vehicle.mode = "RTL"

vehicle.close()
time.sleep(1)
