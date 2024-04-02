import pynmea2

# Read the GPS data from the file
file_path = 'gps.txt'
with open(file_path, 'r') as file:
    for line in file:
        # Check if the line starts with '$' (indicating an NMEA sentence)
        # print(line)
        line = line[2:-2].strip()
        #print(line)
        if line.startswith("$"):
            try:
                # Parse the NMEA sentence
                #print(line)
                msg = pynmea2.parse(line)
                #print(msg)
                if isinstance(msg, pynmea2.types.talker.RMC):                    
                    latitude = msg.latitude
                    longitude = msg.longitude
                    print(f'Latitude: {latitude}, Longitude: {longitude}')
            except pynmea2.ParseError as e:
                pass
                #print(f'Parse error: {e}')
