from gps import gps, WATCH_ENABLE

# Start the GPSD session
session = gps(mode=WATCH_ENABLE)

def get_latest_coords():
    """Fetch the latest GPS coordinates (latitude, longitude) from gpsd."""
    while True:
        try:
            report = session.next()
            if report['class'] == 'TPV':
                lat = getattr(report, 'lat', None)
                lon = getattr(report, 'lon', None)
                if lat is not None and lon is not None:
                    return lat, lon
        except Exception:
            continue