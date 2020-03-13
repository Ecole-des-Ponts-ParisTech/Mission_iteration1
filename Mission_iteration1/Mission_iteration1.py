#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_import_export.py: 

This example demonstrates how to import and export files in the Waypoint file format 
(http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format). The commands are imported
into a list, and can be modified before saving and/or uploading.

Documentation is provided at http://python.dronekit.io/examples/mission_import_export.html
"""
#On importe d'abord les différents modules
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
from pymavlink import mavutil


#On se connecte au drone, et s'il n'y a pas de drone, ce dernier est simulé par SITL
import argparse



def millis():
    return int(round(time.time() * 1000))

def getDistanceBetweenTwoPoints(X1,Y1, X2, Y2):
    dx = X2 - X1
    dy = Y2 - Y1
    return math.sqrt(dx**2 +dy**2)

def convertGlobalToMetric(currentLat, deltaLat, deltaLon):
    """
    Converts a distance measured in latitude and longitude difference metric distance along the North and East direction. 
    This conversion depends on which latitude the drone is located
    Params :
    - currentLat : the current latitude at which the drone is operatin (in degrees)
    - deltaLat : the latitude distance that needs to be converted (in degrees)
    - deltaLon : the longitude distance that needs to be converted (in degrees)

    Outputs: returns the converted distances in mm along the North and East directions
    """
    latitudeToMM, longitudeToMM = getGeoToMetricCoef(currentLat);
    return deltaLat * latitudeToMM, deltaLon * longitudeToMM

def converMetricToGlobal(currentLat, deltaNorth, deltaEast):
    """
    Converts a distance measured in millimeters along the North and East directions into latitude and longitude distances. 
    This conversion depends on which latitude the drone is located
    Params :
    - currentLat : the current latitude at which the drone is operatin (in degrees)
    - deltaNorth : the distance along the North directions that needs to be converted to latitude (in mm)
    - deltaEast : the distance along the East directions that needs to be converted to longitude (in mm)

    Outputs: returns the converted distances in latitude and longitude in degrees
    """
    latitudeToMM, longitudeToMM = getGeoToMetricCoef(currentLat);
    return deltaNorth / latitudeToMM, deltaEast / longitudeToMM

def getGeoToMetricCoef(currentLat):
     # current latitude converted in radians
    radianLattitude = currentLat * math.pi / 180;

    # factor converting 1E-7 degres of latitude to the equivalent in mm at a given latitude
    latitudeToMM = (111132.92 - 559.82 * math.cos(2 * radianLattitude) + 1.175 * math.cos(4 * radianLattitude) - 0.0023*math.cos(6*radianLattitude)) * 1000;
    
    # factor converting 1E-7 degres of latitude to the equivalent in mm at a given latitude
    longitudeToMM = (111412.84 * math.cos(radianLattitude) - 93.5 * math.cos(3*radianLattitude) - 0.118*math.cos(5*radianLattitude))* 1000;

    return latitudeToMM, longitudeToMM

def toRad(degrees):
    return degrees * math.pi / 180

def toDeg(radians):
    return int(radians * 180 / math.pi)


# Deprecated class, only usefull for tests and simulation
class fakePosition(object):
     def __init__(self):
         self.lastPosition = 0
         self.destLat = -35.3628498;
         self.destLon = 149.1635925;
         self.destHeading = 263;
         self.newPosition = False
         
     def getFakePos(self, droneLocation):
        """
        For testing purposes only : recreates a X,Y position relative to a point on the map to simulate a local 
        positionin system
        """       

        deltaLat = droneLocation.lat - self.destLat;
        deltaLon = droneLocation.lon - self.destLon; 
        Xnorth, Yeast = convertGlobalToMetric(self.destLat, deltaLat, deltaLon)
        X = Xnorth * math.cos(self.destHeading * math.pi/180) + Yeast * math.sin(self.destHeading * math.pi/180)
        Y = Xnorth * math.sin(self.destHeading * math.pi/180) - Yeast * math.cos(self.destHeading * math.pi/180)

        return int(X), int(Y)
    
     def isNewPositionAvailable(self):
        if(millis() - self.lastPosition > 100):
            self.lastPosition = millis()
            return True
        else:
            return False








parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#On lance SITL
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


#On se connecte au drone
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

#On definit toutes les fonctions nécessaires à la mission : conversion des latitudes et longitudes en distances, importation des waypoints...

#Fonction retourne une position finale(latitude, longitude, altitude) à partir d'une position initiale et d'un déplacement (distance vers le nord, distance vers l'est)
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

#Fonction qui retourne une distance entre deux positions
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


#Fonction qui retourne la distance à parcourir jusqu'au prochain waypoint
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
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


#Fonction qui retourne des waypoints dans une liste à partir d'un fichier texte de waypoints
def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=float(linearray[0])
                ln_currentwp=float(linearray[1])
                ln_frame=float(linearray[2])
                ln_command=float(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=float(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

#Fonction qui permet d'uploader la liste de waypoints et de la transformer en commande
def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

#Fonction qui retourne la liste des waypoints de la mission réalisée
def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)
        
#Fonction qui affiche le fichier texte des waypoints dans le terminal        
def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())        


import_mission_filename = 'WP_list.txt'
export_mission_filename = 'exportedmission.txt'

#Fonction qui permet au drone de démarrer ses moteurs et de décoller
#En théorie, la commande dronekit qui permet de faire décoller le drone est exécutée, puis le reste du script est lu immédiatement sans laisser le temps au drone de décoller jusqu'à l'altitude demandée
#Il est nécessaire d'ajouter une boucle (while True) qui tourne avec un temps de latence (time.sleep) entre chaque itération, tant que le drone n'a pas attaint 95% de son altitude visée 
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)




#On passe maintenant à la partie du code qui exécute la mission en s'appuyant sur les fonctions définies auparavant

#On upload la mission
upload_mission(import_mission_filename)

#On sauvegarde la mission
save_mission(export_mission_filename)

#On lance les moteurs et on décolle à une altitude donnée
arm_and_takeoff(10)

print("Starting mission")
# On initialise l'indice du waypoint à atteindre
vehicle.commands.next=0

# On passe le drone en mode de vol AUTO : le drone suit automatiquement les waypoints uploadés
vehicle.mode = VehicleMode("AUTO")

# On fait tourner cette boucle While en permanence pour afficher la distance jusqu'au prochain waypoint
# On impose à cette boucle de s'arrêter lorsque le dernier waypoint est atteint
#c'est dans cette foucle qu'il faudrait intégrer l'évitement d'obstacle.
#Lorsque l'on détecte un obstacle, on peut passer en mode LOITER (vol stationnaire), puis en mode GUIDED jusqu'à trouver un waypoint qui permette d'éviter l'obstacle
#On repasse en mode AUTO

F=fakePosition()

while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    if nextwaypoint==6: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (6)")
        break;
    X,Y=F.getFakePos(vehicle.location.global_frame)
    DistanceObstacle = getDistanceBetweenTwoPoints(0,0, X, Y)/1000
    if DistanceObstacle < 2:
        if vehicle.mode == VehicleMode("MANUAL"):
            sys.exit()
        print('Switch to GUIDED mode')
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.velocity[0] = 0
        vehicle.velocity[1] = 0
        vehicle.velocity[2] = 0
        time.sleep(10)
        
        print('Start avoidance')
        DeltaLat1, DeltaLong1 = converMetricToGlobal(vehicle.location.global_frame.lat,0,2000)
        PointEvit1 = LocationGlobalRelative(vehicle.location.global_frame.lat-DeltaLat1, vehicle.location.global_frame.lon-DeltaLong1,10)
        vehicle.simple_goto(PointEvit1)
        time.sleep(15)
        DeltaLat2, DeltaLong2 = converMetricToGlobal(vehicle.location.global_frame.lat,5000,0)
        PointEvit2 = LocationGlobalRelative(vehicle.location.global_frame.lat-DeltaLat2, vehicle.location.global_frame.lon-DeltaLong2,10)
        vehicle.simple_goto(PointEvit2)
        time.sleep(15)
        vehicle.mode = VehicleMode("AUTO")
    print('Distance_Obstacle = ',DistanceObstacle)
    time.sleep(1)


#Lorsque la mission est finie, on passe en mode de vol RTL pour retourner au point de départ et se poser
print('Return to launch')
vehicle.mode = VehicleMode("RTL")

#On arrête le drone
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()


print("\nShow original and uploaded/downloaded files:")
#Print original file (for demo purposes only)
printfile(import_mission_filename)
#Print exported file (for demo purposes only)
printfile(export_mission_filename)
