import os
from gps import *
from time import *
import time
import threading
import math
import json

gpsd = None  # set global variable to access gps

nearby_aircrafts = []

locationHistory = []


class GpsPoller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        global gpsd  # bring it in scope
        gpsd = gps(mode=WATCH_ENABLE)  # starting the stream of info
        self.current_value = None
        self.running = True  # setting the thread running to true

    def run(self):
        global gpsd
        while gpsp.running:
            gpsd.next()  # this will continue to loop and grab EACH set of gpsd info to clear the buffer

class Location:
    lat = None
    lon = None
    time = None
    alt = None
    vel = None
    heading  = None
    climbRate = None
    errorLat = None
    errorLon = None
    errorHorVel = None
    errorVertVel = None
    errorHor = None
    errorVert = None

    def __init__(self, latIn, lonIn, timeIN, altIN, velIN, headIN, climbRateIN, errorLatIN, errorLonIN, errorHorVelIN, errorVertVelIN, errorHorIN, errorVertIN):
        self.lat = latIn
        self.lon = lonIn
        self.time = timeIN
        self.alt = altIN
        self.vel = velIN
        self.heading = headIN
        self.climbRate = climbRateIN
        self.errorLat = errorLatIN
        self.errorLon = errorLonIN
        self.errorHorVel = errorHorVelIN
        self.errorVertVel = errorVertVelIN
        self.errorHor = errorHorIN
        self.errorVertIN = errorVertVelIN


class Aircraft:         # [quick description] -                             [VAR TYPE]
    hex = None          # hex for aircraft identifier ICAO -                STRING
    flight = None       # N-number reg for aircraft -                       STRING
    lat = None          # Latitude (should be decoded from CPR format) -    FLOAT
    lon = None          # Longitude (should be decoded from CPR format) -   FLOAT
    NUCp = None         # Navigation Uncertainty Category -                 INT
    seen_pos = None      # Seconds since last position field update -        DOUBLE
    altitude = None     # Altitude decoded from CPR -                       INT
    vertRate = None     # vertical climb rate -                             INT
    track = None        # unsure -                                          INT
    speed = None        # speed of aircraft                                 INT
    category = None     # aircraft category used to identify size           STRING
    mlat = None         # multilateration msgs (similar to MODE S ?) -      ARRAY
    tisb = None         # TIS-B traffic msgs -                              ARRAY
    messages = None     # number of received messages per aircraft -        INT
    seen = None         # time since last message received (in s?) -        DOUBLE
    rssi = None         # signal strength indicator -                       DOUBLE

    def __init__(self, hexIN, flightIN, latIN, lonIN, NUCpIN, seen_posIN, altitudeIN, vertRateIN, trackIN, speedIN, categoryIN, mlatIN, tisbIN, messagesIN, seenIN, rssiIN):
        self.hex = hexIN
        self.flight = flightIN
        self.lat = latIN
        self.lon = lonIN
        self.NUCp = NUCpIN
        self.seen_pos = seen_posIN
        self.altitude = altitudeIN
        self.vertRate = vertRateIN
        self.track = trackIN
        self.speed = speedIN
        self.category = categoryIN
        self.mlat = mlatIN
        self.tisb = tisbIN
        self.messages = messagesIN
        self.seen = seenIN
        self.rssi = rssiIN

    def update(self, latIN, lonIN, NUCpIN, seen_posIN, altitudeIN, vertRateIN, trackIN, speedIN, categoryIN, mlatIN, tisbIN, messagesIN, seenIN, rssiIN):
        self.lat = latIN
        self.lon = lonIN
        self.NUCp = NUCpIN
        self.seen_pos = seen_posIN
        self.altitude = altitudeIN
        self.vertRate = vertRateIN
        self.track = trackIN
        self.speed = speedIN
        self.category = categoryIN
        self.mlat = mlatIN
        self.tisb = tisbIN
        self.messages = messagesIN
        self.seen = seenIN
        self.rssi = rssiIN


class JSONparser:
    def __init__(self):
        with open('aircrafts.json') as json_file:
            data = json.load(json_file)
            for p in data['aircraft']:
                hexVal = p['hex']

                flightStr = None
                if 'flight' in p:
                    flightStr = p['flight']

                latVal = None
                if 'lat' in p:
                    latVal = p['lat']

                longVal = None
                if 'lon' in p:
                    longVal = p['lon']

                NUCpVal = None
                if 'nucp' in p:
                    NUCpVal = p['nucp']

                seen_posVal = None
                if 'seen_pos' in p:
                    seen_posVal = p['seen_pos']

                altVal = None
                if 'altitude' in p:
                    altVal = p['altitude']

                vert_rateVal = None
                if 'vert_rate' in p:
                    vert_rateVal = p['vert_rate']

                trackVal = None
                if 'track' in p:
                    trackVal = p['track']

                velocity = None
                if 'speed' in p:
                    velocity = p['speed']

                catVal = None
                if 'category' in p:
                    catVal = p['category']

                mlatArr = p['mlat']
                tisbArr = p['tisb']
                msgCount = p['messages']
                lastSeen = p['seen']
                rssiVal = p['rssi']
                for x in nearby_aircrafts:
                    if x.hex == hexVal:
                        x.update(latVal, longVal, NUCpVal, seen_posVal, altVal, vert_rateVal, trackVal, velocity, catVal, mlatArr, tisbArr, msgCount, lastSeen, rssiVal)
                        return None
                tempAircraft = Aircraft(hexVal, flightStr, latVal, longVal, NUCpVal, seen_posVal, altVal, vert_rateVal, trackVal, velocity, catVal, mlatArr, tisbArr, msgCount, lastSeen, rssiVal)
                nearby_aircrafts.append(tempAircraft)


if __name__ == '__main__':
    gpsp = GpsPoller()  # create the thread
    try:
        gpsp.start()  # start it up
        while True:
            # It may take a second or two to get good data
            # print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc

            os.system('clear')

            # set class variables that are needed for ADS-B messages
            lat = gpsd.fix.latitude
            lon = gpsd.fix.longitude
            t = gpsd.fix.time
            alt = gpsd.fix.altitude
            vel = gpsd.fix.speed
            head = gpsd.fix.track  # heading, degrees from true North (0 or 360 degrees), 90 degrees for East, ect.
            climbRate = gpsd.fix.climb  # climb (positive) or sink (negative) rate (m/s)
           # timeUTC = gpsd.utc  # Time in UTC - NOT BEING USED
            errorLat = gpsd.fix.epy  # error estimate of latitude
            errorLong = gpsd.fix.epx  # error estimate of longitude
            errorHorVel = gpsd.fix.eps  # error estimate of speed
            errorVertVel = gpsd.fix.epc  # error estimate of climb/sink in meters per second (Vertical Velocity error)
           # errorHor = gpsd.fix.eph  # estimated horizontal error in meters - WEIRD ERROR
            errorVert = gpsd.fix.epv  # estimated vertical error in meters
			
            print
            print (' GPS reading')
            print ('----------------------------------------')
            print ('latitude    ' , gpsd.fix.latitude)
            print ('longitude   ' , gpsd.fix.longitude)
            print ('time utc    ' , gpsd.utc,' + ', gpsd.fix.time)
            print ('altitude (m)' , gpsd.fix.altitude)
            print ('eps         ' , gpsd.fix.eps) #speed error estimate (m/s), 95% confidence
            print ('epx         ' , gpsd.fix.epx) #longitude error estimate (m), 95% confidence
            print ('epy         ' , gpsd.fix.epy) #latitude error estimate (m), 95% confidence
            print ('epv         ' , gpsd.fix.epv) #estimated vertical error (m), 95% confidence
            print ('ept         ' , gpsd.fix.ept) #estimated timestamp error (% of a second), 95% confidence
            print ('speed (m/s) ' , gpsd.fix.speed)
            print ('climb       ' , gpsd.fix.climb) #climb (positive) or sink (negative) rate, (m/s)
            print ('track       ' , gpsd.fix.track) #course over ground, degrees from true north
            print ('mode        ' , gpsd.fix.mode) #NMEA mode (not sure what this represents)
            print
            print ('sats        ' , gpsd.satellites)
 
            locationHistory.append(Location(lat, lon, t, alt, vel, head, climbRate, errorLat, errorLong, errorHorVel, errorVertVel, None, errorVert))

            if len(locationHistory) > 10:
                locationHistory.pop(0)

            JSONparser()
            for x in nearby_aircrafts:
                if x.seen >= 300:
                    nearby_aircrafts.remove(x)
			# TODO - Search through JSON - if aircraft no longer exists then dump1090 has gotten rid of it and the lastSeen value wont increment through Aircraft.update() so this case will never be 				  reached

            # TODO - Assign aircraft radius, calculate angle and distance between our aircraft and nearby, determine if they are within our radius, and then call method for situation cases


    except(KeyboardInterrupt, SystemExit):  # when you press ctrl+c

        print("\nKilling Thread...")
        gpsp.running = False
        gpsp.join()  # wait for the thread to finish what it's doing
        print("Done.\nExiting.")
