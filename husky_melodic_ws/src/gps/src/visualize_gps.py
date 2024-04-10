import folium

# 42.2736785,-71.80676166666667 (top of unity hall)
# Create a map centered at a specific latitude and longitude

lat, lon = 42.27378197602934, -71.80907628895035 # first coordinate of quad (could make it center of quad)
mymap = folium.Map(location=[lat, lon], zoom_start=24)

with open('quad.txt', 'r') as file:
    for line_num,line in enumerate(file):
        lat, lon = line.split(',')
        # Add a marker at a specific latitude and longitude
        folium.Marker(location=[lat, lon],
        popup="coordinate {x}".format(x = line_num+1)).add_to(mymap)

        # print(f"{line_num}: {lat}, {lon}")


# Save the map as an HTML file
mymap.save("mymap.html")
