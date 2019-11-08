import os
import imageio

png_dir = '/Users/TJLW/Projects/ROS-Navigation/WaypointGraphs/Images/TestEnvironments/Environment1/3x3/WaypointGeneration/StitchedImages'
images = []
for file_name in sorted(os.listdir(png_dir)):
    print(file_name)
    if file_name.endswith('.png'):
        file_path = os.path.join(png_dir, file_name)
        images.append(imageio.imread(file_path))
imageio.mimsave('/Users/TJLW/Projects/ROS-Navigation/WaypointGraphs/Images/TestEnvironments/Environment1/3x3/WaypointGeneration/StitchedImages/movie.gif', images, fps=1)
