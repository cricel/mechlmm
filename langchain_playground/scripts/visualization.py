import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
from mpl_toolkits.mplot3d import Axes3D

# Load map data from YAML file
map_metadata = {
    'image': 'map.pgm',
    'resolution': 0.05,  # Each pixel represents 0.05 meters
    'origin': [-1.17, -4.35, 0]  # Map origin coordinates in meters
}

# Update the map image path to the new location
map_image_path = '../data/images/map.pgm'
map_image = imread(map_image_path)

# Sample database with positions relative to the map
data = [
    {"name": "person", "position": [1, 2, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_1"},
    {"name": "phone", "position": [2, 2, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_2"},
    {"name": "chair", "position": [1, 1, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_3"},
    {"name": "cloth", "position": [1, 2, 1], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_1"},
    {"name": "desk", "position": [1, 2, 1], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_1"}
]

# Adjust the item positions relative to the map origin and resolution
def adjust_position(position, resolution, origin):
    adjusted_x = (position[0] - origin[0]) / resolution
    adjusted_y = (position[1] - origin[1]) / resolution
    return [adjusted_x, adjusted_y, position[2]]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the map in 3D as a surface (flat at z=0)
map_height, map_width = map_image.shape
x = np.linspace(0, map_width, map_width)
y = np.linspace(0, map_height, map_height)
x, y = np.meshgrid(x, y)

# Convert the map image to a format suitable for displaying as a surface in grayscale
map_surface = np.flipud(map_image)  # Flip the image so it aligns with the origin

# Plot the map as a 3D surface at z=0
ax.plot_surface(x, y, np.zeros_like(map_surface), rstride=1, cstride=1, facecolors=plt.cm.gray(map_surface / 255), shade=False)

# Extract the data for plotting and place items above the map
for item in data:
    adjusted_position = adjust_position(item["position"], map_metadata['resolution'], map_metadata['origin'])
    
    # Scatter the items above the map (z-axis is the third position element)
    ax.scatter(adjusted_position[0], adjusted_position[1], adjusted_position[2], marker='o', s=100, label=item["name"])
    
    # Annotate each item with its name and notes
    ax.text(adjusted_position[0], adjusted_position[1], adjusted_position[2], f"{item['name']}\n{item['notes']}", fontsize=9)

# Set labels and view angle
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.view_init(elev=90, azim=-90)  # Top-down view for better visualization

# Show the plot with a legend
plt.legend()
plt.show()
