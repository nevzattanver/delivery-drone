

from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
from detection_delivery_point import detect

################################################################################################
# Ayarlar
################################################################################################

connection_string       = '///dev/ttyAMA0:57600' 
MAV_MODE_AUTO   = 4 


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Başlangıç Ayarları ve Tanımlamalar
################################################################################################

print("Hava Aracina Baglaniliyor...")
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode): 
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def get_location_offset_meters(original_location, dNorth, dEast, alt):                                                                     
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt) 
    
    
def get_distance_metres(aLocation1, aLocation2):
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
# Dinleyiciler
################################################################################################

home_position_set = False
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True


################################################################################################
# Görev Başlangıcı
################################################################################################

while not home_position_set:
    print "GPS Ev Pozisyon Kilitlenmesi Gerceklesitiriliyor..."
    time.sleep(1)

print(" Arac Tipi: %s" % vehicle._vehicle_type)
print(" Arm Durumu: %s" % vehicle.armed)
print(" Sistem Durumu: %s" % vehicle.system_status.state)
print(" GPS Durumu: %s" % vehicle.gps_0)
print(" Irtifa: %s" % vehicle.location.global_relative_frame.alt)

PX4setMode(MAV_MODE_AUTO)
time.sleep(1)
vehicle.groundspeed=3
vehicle.airspeed=3

cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

flagIsSelected = False
while not flagIsSelected:
    print("1.Gazi Universitesi YDYO Bahcesi \n")
    print("2.Gazi Universitesi GSF Bahcesi \n")
    choice=input("Lutfen gitmek istediginiz hedefi seçiniz \n")
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
        print("Yanlis giris yapildi...")
    
    
print "Gorev Baslatiliyor... Gorevler Oto Pilot Kartina Gonderiliyor..."      
wp = get_location_offset_meters(home, 0, 0, 30);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, yaw_heading, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)


cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, yaw_heading, 
arrival_lat, arrival_lon, wp.alt)
cmds.add(cmd)

cmds.upload()
time.sleep(2)

# Arm vehicle
print "Hava Araci ARM Moduna Aliniyor..."
vehicle.armed = True

print "Kalkis Gerceklesiyor..."
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print "Hedef Koordinata Gidiliyor..."
        nextwaypoint = vehicle.commands.next
    time.sleep(1)
print " Teslimat Bolgesine Kalan Mesafe : %s" % distance_to_current_waypoint()

while distance_to_current_waypoint() > 0.1:	
	time.sleep(1)



wp = get_location_offset_meters(vehicle.location.global_relative_frame, 0, 0, -25);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
time.sleep(2)
nextwaypoint = vehicle.commands.next



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
        print("Yuk birakilmaya hazirlaniyor...")
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

servo_msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            1,  # servo number
            2000,  # servo position between 1000 and 2000
            0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    
time.sleep(10)
vehicle.send_mavlink(servo_msg)

time.sleep(10)
vehicle.mode = "RTL"

vehicle.close()
time.sleep(1)
