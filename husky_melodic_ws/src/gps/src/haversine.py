from math import radians, sin, cos, sqrt, atan2, acos, degrees


def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = 6371 * c  # Radius of Earth in kilometers
    return distance * 1000


def law_cosine_angle(a, b, c):
    term_1 = (c**2 - b**2 - a**2)
    term_2 = -2.0 * a * b
    #print(term_1)
    # print(term_2)
    return degrees(acos(term_1/(term_2)))


def simulate_angular_motion():
    total_angle = 0.0
    points_traveled = 0.0
    # you need to store the p2_lat and p2_lon as previous points
    with open('quad.txt', 'r') as file:
        for line_num, line in enumerate(file):
            if line_num == 0:
                # in the beginning, we are moving from point 0 to point 1
                origin_lat, origin_lon = map(float, line.split(','))
                p1_lat, p1_lon = map(float, file.readline().strip().split(','))
                
                dist = haversine(origin_lat, origin_lon, p1_lat, p1_lon)
                angle = acos(1/dist)
                print("starting motion angle: ", angle)
                total_angle = total_angle + angle
                points_traveled = points_traveled + 1
                print(points_traveled)
            else:
                try:
                    p2_lat, p2_lon = map(float, file.readline().strip().split(','))

                    p1_origin = haversine(origin_lat, origin_lon, p1_lat, p1_lon)
                    p2_origin = haversine(origin_lat, origin_lon, p2_lat, p2_lon)
                    p1_p2 = haversine(p1_lat, p1_lon, p2_lat, p2_lon)
                    gamma = law_cosine_angle(p1_origin, p2_origin, p1_p2)
                    total_angle = total_angle + gamma
                    points_traveled = points_traveled + 1
                    print("gamma:", gamma, "degrees")
                    p1_lat, p1_lon = p2_lat, p2_lon

                    print(points_traveled)
                except ValueError as e:
                    pass
                    #print(e)
                    

    print("total angle traveled: ", total_angle, "with ", points_traveled, " points traveled")


simulate_angular_motion()



simulate_angular_motion()
# print("distance between coordinates:", dist, " meters")
