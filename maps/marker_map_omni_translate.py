import yaml
import math

# Load the YAML data from a file
data = None
with open("marker_positions.yml", 'r') as file:
    data = yaml.safe_load(file)


# The number to add to the coordinates
num_to_add = 1

for marker in data['marker_positions']:
    # Add the number to the coordinates
    marker['x'] += 6.807226
    marker['y'] -= 5.166632

    # Convert to degrees
    marker['roll'] = -math.degrees(marker['roll'])
    marker['pitch'] = 0
    marker['yaw'] = 0

# Convert the data back to a YAML string
yaml_str_updated = yaml.dump(data)

print(yaml_str_updated)