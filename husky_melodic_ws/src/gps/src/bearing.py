from math import radians, degrees, sin, cos, atan2


def initial_bearing(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Calculate the bearing
    dlon = lon2 - lon1
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    y = sin(dlon) * cos(lat2)
    initial_bearing = atan2(y, x)

    # Convert bearing from radians to degrees
    #initial_bearing = degrees(initial_bearing)

    # Normalize the bearing to a compass bearing (between 0 and 360 degrees)
    #compass_bearing = (initial_bearing + 360) % 360

    return initial_bearing


lat1, lon1 = 42.273424, -71.809441  # Gompei Statue
lat2, lon2 = 42.273438, -71.809523    # next point
bearing = initial_bearing(lat1, lon1, lat2, lon2)
print("Initial Bearing:", bearing, "radians")
