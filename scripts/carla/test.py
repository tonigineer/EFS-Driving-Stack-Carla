#!/usr/bin/python3

import carla
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import json
import os

def extract_lane_boundaries_from_map(world, road_dic):
    # Get the map data
    map_data = world.get_map()

    # Extract lane boundaries
    right_boundaries = []
    left_boundaries = []

    right_boundaries_long_lat = []
    left_boundaries_long_lat = []
    a = map_data.generate_waypoints(20.0)
    for waypoint in map_data.generate_waypoints(20.0):
        if (waypoint.road_id in road_dic): #and (waypoint.lane_id==road_dic[waypoint.road_id][0]) :
        #if True:
            left_marking = waypoint.get_left_lane()
            right_marking = waypoint.get_right_lane()
            
            if left_marking and right_marking:
                if (left_marking.lane_id == road_dic[waypoint.road_id][0]) and (right_marking.lane_id == road_dic[waypoint.road_id][1]) :
                    left_point = left_marking.transform.location
                    right_point = right_marking.transform.location

                    right_boundaries.append((right_point, right_marking.road_id, right_marking.lane_id))
                    left_boundaries.append((left_point, left_marking.road_id, left_marking.lane_id))

                    right_boundaries_long_lat.append(map_data.transform_to_geolocation(right_point))
                    left_boundaries_long_lat.append(map_data.transform_to_geolocation(left_point))

    return right_boundaries, left_boundaries, right_boundaries_long_lat, left_boundaries_long_lat

# Get the directory of the script
script_directory = os.path.dirname(os.path.abspath(__file__))

# Change the current working directory to the script directory
os.chdir(script_directory)

windows_host = os.environ.get("WINDOWS_HOST")
client = carla.Client(windows_host, 2000)
client.set_timeout(10.0)
world = client.get_world()

# Define the roads and lanes from which we want to extract the boundaries:
road_dic = {
    6:[1,3],
    90:[1,3],
    7:[1,3],
    17:[-1,-3],
    10:[-1,-3],
    0:[-1,3],
    3:[-1,-3],
    566:[-1,-3],
    675:[-1,-3],
    1:[-1,-3],
    8:[-1,-3],
    4:[1,3],
    516:[1,3],
    5:[1,3],
    735:[1,3]
}


# Extract lane boundaries
right_boundaries, left_boundaries, right_boundaries_long_lat, left_boundaries_long_lat = extract_lane_boundaries_from_map(world, road_dic)

print (len(left_boundaries))
print (len(right_boundaries))

##### json file #####

# Create empty lists for JSON structure
innerLong = []
innerLat = []
outerLong = []
outerLat = []

# Iterate over left and right boundaries simultaneously
for left_boundary, right_boundary in zip(left_boundaries_long_lat, right_boundaries_long_lat):
    # Extract longitude and latitude for left boundary
    innerLong.append(left_boundary.longitude)
    innerLat.append(left_boundary.latitude)

    # Extract longitude and latitude for right boundary
    outerLong.append(right_boundary.longitude)
    outerLat.append(right_boundary.latitude)

# Create the JSON dictionary
json_data = {
    "outerLong": outerLong,
    "innerLong": innerLong,
    "innerLat": innerLat,
    "outerLat": outerLat
}

# Write the JSON data to a file
with open("boundary_data.json", "w") as file:
    json.dump(json_data, file, indent=4, separators=(",",":"))

#### plot #####
plt.title("Lane Boundaries")
plt.xlabel("X")
plt.ylabel("Y")

# Plot the lane boundaries
for left_boundary  in left_boundaries:
    left_x = left_boundary[0].x
    left_y = left_boundary[0].y
    plt.annotate(str(left_boundary[1])+"/"+str(left_boundary[2]), xy=(left_x, left_y) )
    plt.scatter(left_x, left_y, color='red')

for right_boundary  in right_boundaries:
    right_x = right_boundary[0].x
    right_y = right_boundary[0].y
    plt.annotate(str(right_boundary[1])+"/"+str(right_boundary[2]), xy=(right_x, right_y) )
    plt.scatter(right_x, right_y, color='blue')

# Save the plot to a file
plt.savefig("lane_boundaries.png")
