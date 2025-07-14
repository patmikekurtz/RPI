from gps import gps, WATCH_ENABLE
import time

session = gps(mode=WATCH_ENABLE)

def latestLat():
    try:
        report = session.next()
        if report['class'] == 'TPV':
            myLat = getattr(report, 'lat', None)
    return myLat

def latestLon():
    try:
        report = session.next()
        if report['class'] == 'TPV':
            myLon = getattr(report, 'lat', None)
    return myLon

while True:
    try:
        report = session.next()
        if report['class'] == 'TPV':
            lat = getattr(report, 'lat', None)
            lon = getattr(report, 'lon', None)

            if lat is not None and lon is not None:
                maps_url = f"https://www.google.com/maps?q={lat},{lon}"
                print(f"Latitude: {lat}, Longitude: {lon}")
                print(f"Google Maps: {maps_url}\n")

        time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
        break