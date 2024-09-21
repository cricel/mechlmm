import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sample database
data = [
    {"name": "item_1", "position": [1, 2, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_1"},
    {"name": "item_2", "position": [2, 2, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_2"},
    {"name": "item_3", "position": [1, 1, 2], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_3"},
    {"name": "item_4", "position": [1, 2, 1], "features": ["features_1", "features_2", "features_3"], "timestamp": "12:3:21", "notes": "note_1"}
]

# Extract the data for plotting
x_coords = [item["position"][0] for item in data]
y_coords = [item["position"][1] for item in data]
z_coords = [item["position"][2] for item in data]
names = [item["name"] for item in data]
notes = [item["notes"] for item in data]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
sc = ax.scatter(x_coords, y_coords, z_coords, c='b', marker='o')

# Annotate points with names and notes
for i, name in enumerate(names):
    ax.text(x_coords[i], y_coords[i], z_coords[i], f"{name}\n{notes[i]}")

# Set labels
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')

# Show the plot
plt.show()
