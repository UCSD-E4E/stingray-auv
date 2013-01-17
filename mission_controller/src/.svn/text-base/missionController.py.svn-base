#! /usr/bin/env python
from pylab import *
import pylab
import csv
import roslib;
roslib.load_manifest('mission_controller')
from missionController.msg import TargetWaypoints
from stateEstimator.msg import State
import rospy
import sys
import os
#from scipy import *
from pylab import *

# For opening from launch file.
if len(sys.argv) < 1:
    filename = rospy.get_param('filename')
    rospy.loginfo("filename = %s", filename)
else:
    filename = os.path.join(sys.path[0], sys.argv[1])
    rospy.loginfo("filename = %s", filename)
fWaypoints = open(filename, "rU")
readerWaypoints = csv.DictReader(fWaypoints)

red = "ff0000ff"
blue = "ffff0000"

# Declare vectors to hold data. t_ stands for 'target'.
t_surge = []
t_lat = []
t_lon = []
t_depth = []

# Declare variables to store where we have been.
log_surge = []
log_lat = []
log_lon = []
log_depth = []

# Read and store data from log file in vectors using labels on first line of log file.
for data in readerWaypoints:
    t_surge.append(str(data["m_surge"]))
    t_lat.append(str(data["m_lat"]))
    t_lon.append(str(data["m_lon"]))
    t_depth.append(str(data["m_depth"]))

numWaypoints = len(t_lat)
initialHeading = str(4.71238898) # 3pi/2, come up with a smarter way to initialize the initial heading!

#Open KML Logger files
log_waypoints_kml = open('gps_target_data.kml','w')
log_waypoints_kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>" + '\n')
log_waypoints_kml.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">" + '\n')
log_waypoints_kml.write("<Document>" + '\n')
log_waypoints_kml.write("<name>" + "KML_Target_GPS_Data" + "</name>" + '\n')
log_waypoints_kml.write("<open>1</open>" + '\n')
log_waypoints_kml.write("<LookAt>" + '\n')
log_waypoints_kml.write("<longitude>" + t_lon[0] + "</longitude>" + '\n')
log_waypoints_kml.write("<latitude>" + t_lat[0] + "</latitude>" + '\n')
log_waypoints_kml.write("<altitude>1</altitude>" + '\n')
log_waypoints_kml.write("<range>150</range>" + '\n')
log_waypoints_kml.write("<tilt>0</tilt>" + '\n')
log_waypoints_kml.write("<heading>" + initialHeading + "</heading>" + '\n')
log_waypoints_kml.write("</LookAt>" + '\n')
log_waypoints_kml.write("<Placemark>" + '\n')
log_waypoints_kml.write("<name>" + "KML_Target_GPS_Data" + "</name>" + '\n')
log_waypoints_kml.write("<Style>" + '\n')
log_waypoints_kml.write("<LineStyle>" + '\n')
log_waypoints_kml.write("<color>" + blue + "</color>" + '\n')
log_waypoints_kml.write("<width>3</width>" + '\n')
log_waypoints_kml.write("</LineStyle>" + '\n')
log_waypoints_kml.write("</Style>" + '\n')
log_waypoints_kml.write("<LineString>" + '\n')
log_waypoints_kml.write("<extrude>1</extrude>" + '\n')
log_waypoints_kml.write("<tessellate>1</tessellate>" + '\n')
log_waypoints_kml.write("<altitudeMode>relativeToGround</altitudeMode>" + '\n')
log_waypoints_kml.write("<coordinates>" + '\n')

log_kml = open('gps_data.kml','w')
log_kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>" + '\n')
log_kml.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">" + '\n')
log_kml.write("<Document>" + '\n')
log_kml.write("<name>" + "KML_GPS_Data" + "</name>" + '\n')
log_kml.write("<open>1</open>" + '\n')
log_kml.write("<LookAt>" + '\n')
log_kml.write("<longitude>" + t_lon[0] + "</longitude>" + '\n')
log_kml.write("<latitude>" + t_lat[0] + "</latitude>" + '\n')
log_kml.write("<altitude>1</altitude>" + '\n')
log_kml.write("<range>150</range>" + '\n')
log_kml.write("<tilt>0</tilt>" + '\n')
log_kml.write("<heading>" + initialHeading + "</heading>" + '\n')
log_kml.write("</LookAt>" + '\n')
log_kml.write("<Placemark>" + '\n')
log_kml.write("<name>" + "KML_GPS_Data" + "</name>" + '\n')
log_kml.write("<Style>" + '\n')
log_kml.write("<LineStyle>" + '\n')
log_kml.write("<color>" + red + "</color>" + '\n')
log_kml.write("<width>3</width>" + '\n')
log_kml.write("</LineStyle>" + '\n')
log_kml.write("</Style>" + '\n')
log_kml.write("<LineString>" + '\n')
log_kml.write("<extrude>1</extrude>" + '\n')
log_kml.write("<tessellate>1</tessellate>" + '\n')
log_kml.write("<altitudeMode>relativeToGround</altitudeMode>" + '\n')
log_kml.write("<coordinates>" + '\n')

# Define missions numerically.
followPath = 1
recharge = 2
mc_debug = 3

# Initialize current mission to -1.
mission = -1

# Initialize variables.
waypointCounter = 0
success = False
MissionIncomplete = True
timeOutCounter = 0
maxTime = 300000
debug_count = 10

# Initialize target values.
target_surge = 0.0
target_lat = 0.0
target_lon = 0.0
target_depth = 0.0

# Used to convert latitude and longitude to meters for waypoint hit determination in meters.
lat2meters = 111131.75
lon2meters = 78846.81

# Initialize actual states.
lat = float(t_lat[0])
lon = float(t_lon[0])
depth = 0.0

# Initialize waypoint hit radius in meters.
waypointRadius = 65.0

# minimum velocity for slowing down as we approach a waypoint
minVelocity = 1.0

while (mission < 0):
    sys.stdout.write("Enter 1 for follow path, Enter 2 for recharge, Enter 3 for MC debug: ")
    mission = int(raw_input())
    if mission > 3 :
        mission = -1

data = State()
def stateCallback(data):
    global lat
    global lon
    global depth
    lat = data.lat
    lon = data.lon
    depth = data.depth
    
    log_lat.append(lat)
    log_lon.append(lon)
    log_kml.write(str(lon) + ',' + str(lat) + str(0.0) + ' ')
    
msg = TargetWaypoints()
pub = rospy.Publisher('targetWaypoints', TargetWaypoints)
rospy.init_node('mission_controller' , log_level=rospy.INFO)
rospy.Subscriber("state", State, stateCallback)
        
def waypointPublisher():
    #while not rospy.is_shutdown():
    msg.target_surge = target_surge
    msg.target_lat = target_lat
    msg.target_lon = target_lon
    msg.target_depth = target_depth
    pub.publish(msg)
    #rospy.loginfo('target_surgePublished = ' +  str(target_surge))
    rospy.sleep(1.0)

# Start of the main part of the script.
while MissionIncomplete :
    rospy.loginfo('Mission In Progress')
    if mission == followPath:
        if waypointCounter < numWaypoints:
            target_surge = float(t_surge[waypointCounter])
            target_lat = float(t_lat[waypointCounter])
            target_lon = float(t_lon[waypointCounter])
            target_depth = float(t_depth[waypointCounter])
            # Compute errors.
            latErrInMeters = (lat-target_lat) * lat2meters
            lonErrInMeters = (lon-target_lon) * lon2meters
            depthErr = depth-target_depth
            
            distance = (latErrInMeters**2 + lonErrInMeters**2 + depthErr**2)**.5
            rospy.loginfo('Distance = ' +  str(distance))
            
            rospy.loginfo('target_latPre = ' +  str(target_lat))
            rospy.loginfo('target_lonPre = ' +  str(target_lon))
            # Crude way of slowing down as we approach, refine later.
            #if waypointCounter < numWaypoints -1 :
				#target_latNext = float(t_lat[waypointCounter+1])
				#target_lonNext = float(t_lon[waypointCounter+1])
				#target_depthNext = float(t_depth[waypointCounter+1])
				
				#latErrInMetersNext = (target_lat-target_latNext) * lat2meters
				#lonErrInMetersNext = (target_lon-target_lonNext) * lon2meters
				#depthErrNext = target_depth - target_depthNext
				 
				#distanceToNextWaypoint = (latErrInMetersNext**2 + lonErrInMetersNext**2 + depthErrNext**2)**.5
				#alphaLat = abs((latErrInMeters**3)) / (abs(((latErrInMeters**3)) + .25 * (waypointRadius**0.333)))
				#alphaLon = abs((lonErrInMeters**3)) / (abs(((lonErrInMeters**3)) + .25 * (waypointRadius**0.333)))
				#alphaDepth = depthErr**3 / (depthErr**3 + .25 * (waypointRadius**0.333))
				
				#rospy.loginfo('alphaLat = ' +  str(alphaLat))
				#rospy.loginfo('alphaLon = ' +  str(alphaLon))
				#rospy.loginfo('lonErrInMeters = ' +  str(lonErrInMeters))
				#target_lat = alphaLat * target_lat + (1-alphaLat) * target_latNext
				#target_lon = alphaLon * target_lon + (1-alphaLon) * target_lonNext
				#target_depth = alphaDepth * target_depth + (1-alphaDepth) * target_depthNext
				
				#rospy.loginfo('target_latPost = ' +  str(target_lat))
				#rospy.loginfo('target_lonPost = ' +  str(target_lon))
				#rospy.loginfo('target_lonNext = ' +  str(target_lonNext))
            try:
                # Publish the targets.
                waypointPublisher()
            except rospy.ROSInterruptException: pass    
            
            #rospy.loginfo('current lat = ' +  str(lat))
            #rospy.loginfo('current lon = ' +  str(lon))
            # Check if we have reached the current waypoint.
            if distance <= waypointRadius :
                success = True
                
            elif timeOutCounter > maxTime :
                success = True
                timeOutCounter = 0
                rospy.loginfo('Waypoint Time Out, Moving On')
                
            else :
                success = False
                timeOutCounter += 1
            
            # Upon success, move on to next waypoint.
            if success:
				log_waypoints_kml.write(t_lon[waypointCounter] + ',' + t_lat[waypointCounter] + str(0.0) + ' ')
				waypointCounter = waypointCounter + 1
				rospy.loginfo(str(waypointCounter))
        else :
            print 'Mission Complete'
            MissionIncomplete = False
        
    elif mission == recharge:
        try:
            # Publish the targets.
            waypointPublisher()
            MissionIncomplete = False
        except rospy.ROSInterruptException: pass

    elif mission == mc_debug:
        while(debug_count > 0):
            try:
                waypointPublisher()
                #rospy.loginfo('current lat = ' +  str(lat))
                #rospy.loginfo('current lon = ' +  str(lon))
                debug_count -=1
            except rospy.ROSInterruptException: pass
        raise SystemExit
        
    else:
        close()
        raise SystemExit

# Display the figures and close out the program.
# Make the plot pretty.
#title("Waypoints vs. Time")
#xlabel("Waypoint Number")
#ylabel("Data")

# Make a legend. This is iffy and depends on the data you are plotting. Probably should leave commented out.
#figlegend((lACSYawState, lGPSYawCalculated), ('ACSYawState', 'GPSYawCalculated'), 'upper right')


#Close KML Files
log_kml.write('\n'+ "</coordinates>" + '\n' + "</LineString>" + '\n' + "</Placemark>" + '\n' + "</Document>" + '\n' + "</kml>" + '\n')
log_kml.close()

log_waypoints_kml.write('\n'+ "</coordinates>" + '\n' + "</LineString>" + '\n' + "</Placemark>" + '\n' + "</Document>" + '\n' + "</kml>" + '\n')
log_waypoints_kml.close()

figure()

hold(True)
plot(log_lon,log_lat)
plot(t_lon, t_lat)
xlabel('longitude')
ylabel('latitude')
show()
close()
raise SystemExit
