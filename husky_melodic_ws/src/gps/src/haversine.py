from math import radians, sin, cos, sqrt, atan2, acos, degrees
from coordinate import GpsCoordinate

# unit: meters
def haversine(reference_coordinate, next_coordinate):
    # Convert latitude and longitude from degrees to radians
    lat1 = reference_coordinate.latitude
    lon1 = reference_coordinate.longitude

    lat2 = next_coordinate.latitude
    lon2 = next_coordinate.longitude

    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = 6371 * c  # Radius of Earth in kilometers
    return distance * 1000

# unit: radians
def law_cosine_angle(a, b, c):
    term_1 = (c**2 - b**2 - a**2)
    term_2 = -2.0 * a * b
    return acos(term_1/(term_2))


def find_closest_point(list_map, ref_coordinate):
    closest_point = None
    # starting closest distance is extremely large to find closer points
    closest_distance = 9e1000
    for idx, coordinate in enumerate(list_map):
        dist = haversine(ref_coordinate, coordinate)
        if dist < closest_distance:
            closest_point = coordinate
            closest_index = idx
            closest_distance = dist

    return closest_index, closest_point


def generate_map(map_filename):
    gps_map = []
    with open(map_filename, 'r') as file:
        for line in file:
            # locate the closest point to where we are currently
            p_lat, p_lon = map(float, line.strip().split(','))
            coordinate = GpsCoordinate(p_lat, p_lon)
            gps_map.append(coordinate)

    return gps_map


def simulate_angular_motion(starting_coordinate):

    map_file = 'quad.txt'
    quad_map = generate_map(map_file)  # list of GpsCoordinate objects

    starting_idx, ref_coordinate = find_closest_point(quad_map, starting_coordinate)
    current_heading = 0
    curr_point = starting_coordinate

    for point in range(1, len(quad_map)):
        # find distance between point where robot is at and the reference point
        next_point = (starting_idx + point) % len(quad_map)
        next_point = quad_map[next_point] # get the next point from the ref map
        oc = haversine(ref_coordinate, curr_point)
        op = haversine(ref_coordinate, next_point)
        cp = haversine(curr_point, next_point)
        phi = law_cosine_angle(cp, oc, op)
        # print(phi)
        delta_theta = phi - current_heading
        print("delta_theta:", delta_theta)
        current_heading += delta_theta

        # distance from current to point        
        distance_to_travel = cp

        # update the robot position to the next point
        curr_point = next_point

        #print(point,":","the current heading is:", current_heading, "distance to travel:", distance_to_travel)



# simulate_angular_motion()
#starting_gps_lat, starting_gps_lon = (42.27375392087034, -71.80908967954097)

#starting_coordinate = GpsCoordinate(starting_gps_lat, starting_gps_lon)
# map_file = 'quad.txt'
# quad_map = generate_map(map_file)  # list of GpsCoordinate object
# idx, coordinate = find_closest_point(quad_map, starting_coordinate)
# print(idx, ":", coordinate.latitude, ",", coordinate.longitude)

#simulate_angular_motion(starting_coordinate)
