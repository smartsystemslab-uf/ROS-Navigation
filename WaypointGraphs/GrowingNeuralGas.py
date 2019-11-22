import os
import sys

import itertools
import glob

import matplotlib.pyplot as plt
import numpy as np

from PIL import Image

from neupy import algorithms, utils
from skimage import color, data, io, img_as_float
from skimage.filters import threshold_otsu, threshold_local, gaussian
from skimage.transform import rescale, resize, downscale_local_mean



CURRENT_DIR = os.path.abspath(os.path.dirname(__name__))
LIBRARY_DIR = os.path.join(CURRENT_DIR, '..', '..')



# Convert each pixel to a separate data point
#   We will use these data points in order to learn
#   topological structure of the image
def image_to_data(img):
    free_space_data = []
    occupied_space_data = []
    for (x, y), value in np.ndenumerate(img):
        # Black pixels = occupied space
        if value == 0:
            occupied_space_data.append([y, -x])

        # White pixels = free space
        else:
            free_space_data.append([y, -x])



    return free_space_data, occupied_space_data

def draw_image(graph, show=True):
    # plt.scatter(*np.array(data).T / scale_factor, s=5, alpha=1);

    for node_1, node_2 in graph.edges:
        weights = np.concatenate([node_1.weight, node_2.weight])
        line, = plt.plot(*weights.T, color='black')
        plt.setp(line, linewidth=0.2, color='black')

    plt.xticks([], [])
    plt.yticks([], [])

    if show:
        # plt.ion()
        plt.pause(0.0001)
        plt.show()

def create_gng(max_nodes, step=0.2, n_start_nodes=2, max_edge_age=50):
    return algorithms.GrowingNeuralGas(
        n_inputs=2,
        n_start_nodes=n_start_nodes,

        shuffle_data=True,
        verbose=False,

        step=step,
        neighbour_step=0.005,

        max_edge_age=max_edge_age,
        max_nodes=max_nodes,

        n_iter_before_neuron_added=100,
        after_split_error_decay_rate=0.5,
        error_decay_rate=0.995,
        min_distance_for_update=0.01,
    )

def gng_status(gng):
    print('\tCurrent node count: ' + str(gng.graph.n_nodes))
    print('\tTotal training updates: ' + str(gng.n_updates_made))

def init_camera_grid(camera_grid_rows, camera_grid_cols):
    rows = [i for i in range(camera_grid_rows)]
    columns = [i for i in range(camera_grid_cols)]
    return {r : {c:None for c in columns} for r in rows}


def save(filepath, fig=None):
    '''Save the current image with no whitespace
    Example filepath: "myfig.png" or r"C:\myfig.pdf"
    '''
    import matplotlib.pyplot as plt
    if not fig:
        fig = plt.gcf()


# Valid transition regions are green
#   Transition points are blue
#   Camera fields of view borders are red
def stitchGNGImagesToGrid(camera_transition_regions, camera_transition_points, output_filename, subview_directory, rows, cols, row_overlap, col_overlap):
    subview_directory = subview_directory

    subview_rows = rows
    subview_cols = cols

    row_overlap_pixels = row_overlap
    col_overlap_pixels = col_overlap

    # PILLOW IMAGES
    subview_list = list(map(Image.open, sorted(glob.glob(subview_directory + '/subview_*'))))
    # print(str(len(subview_list)) + ' subviews found in ' + subview_directory)

    # for im in subview_list:
    #     print(im.filename)

    # All subview sizes exptected to be the same
    subview_width = subview_list[0].size[0]
    subview_height = subview_list[0].size[1]

    output_width = (subview_cols * subview_width) - (col_overlap_pixels * (subview_cols - 1))
    output_height = (subview_rows * subview_height) - (row_overlap_pixels * (subview_rows - 1))

    grid = Image.new('L', (output_width, output_height), 'black').convert('RGB')
    # grid.convert('P')


    # Do the stitch
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            grid.paste(subview_list[subview_index], (offset_x, offset_y))



    # Draw transition regions
    transition_region_color = (0,255,0) # Green
    for transition_regions in camera_transition_regions:
        for border_regions in transition_regions:
            for region in border_regions:
                for x,y in region:
                    grid.putpixel((x, y), transition_region_color)


    # Draw transition points
    transition_point_marking_radius = 3
    transition_point_color = (0,0,255) # Blue
    for transition_points in camera_transition_points:
        for border_transition_points in transition_points:
            for x,y in border_transition_points:
                # Draw centroid marking
                centroid = (int(x), int(y))
                for square_x in range(centroid[0]-transition_point_marking_radius, centroid[0]+transition_point_marking_radius+1):
                    for square_y in range(centroid[1]-transition_point_marking_radius, centroid[1]+transition_point_marking_radius+1):
                        grid.putpixel((square_x, square_y), transition_point_color) # Blue


    # Add borders for each subview such that overlaps can be seen
    border_color = (200,0,0)
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            # Add borders
            border_width = 2
            for x in range(offset_x, offset_x + subview_list[subview_index].width):
                # Top border for each subview
                for y in range(offset_y, offset_y + border_width):
                    grid.putpixel((x, y), border_color)
                # Bottom border for each subview
                for y in range(offset_y + subview_list[subview_index].height - border_width, offset_y + subview_list[subview_index].height):
                    grid.putpixel((x, y), border_color)

            for y in range(offset_y, offset_y + subview_list[subview_index].height):
                # Left border for each subview
                for x in range(offset_x, offset_x + border_width):
                    grid.putpixel((x, y), border_color)
                # Right border for each subview
                for x in range(offset_x + subview_list[subview_index].width - border_width, offset_x + subview_list[subview_index].width):
                    grid.putpixel((x, y), border_color)


    # Finish up
    grid.save(output_filename)



# Find centroid of a region of points
#   Region is a list of points (x,y)
def FindCentroid(region):
    x = [p[0] for p in region]
    y = [p[1] for p in region]
    return (sum(x) / len(region), sum(y) / len(region))



# Possible transition regions in overlapping regions are yellow
#   Valid transition regions are green
#   Transition points are blue
#   Camera fields of view borders are red
def FindTransitionRegions(camera_transition_regions, camera_transition_points, output_filename, subview_directory, rows, cols, row_overlap, col_overlap):
    subview_directory = subview_directory

    subview_rows = rows
    subview_cols = cols

    row_overlap_pixels = row_overlap
    col_overlap_pixels = col_overlap

    # PILLOW IMAGES
    subview_list = list(map(Image.open, sorted(glob.glob(subview_directory + '/*subview*'))))
    print(str(len(subview_list)) + ' subviews found in ' + subview_directory)

    # for im in subview_list:
    #     print(im.filename)

    # All subview sizes exptected to be the same
    subview_width = subview_list[0].size[0]
    subview_height = subview_list[0].size[1]

    output_width = (subview_cols * subview_width) - (col_overlap_pixels * (subview_cols - 1))
    output_height = (subview_rows * subview_height) - (row_overlap_pixels * (subview_rows - 1))

    grid = Image.new('L', (output_width, output_height), 'black').convert('RGB')
    # grid.convert('P')


    # Do the stitch
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            grid.paste(subview_list[subview_index], (offset_x, offset_y))



    # Identify valid transition regions
    #   Looks for free space that is the specified height or width for corresponding top/bottom and left/right transition regions respecitvely
    #   Also eliminates transition regions that are smaller than transition_region_min_pixels
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            top_transition_regions = []
            bottom_transition_regions = []
            left_transition_regions = []
            right_transition_regions = []

            current_top_transition_region = []
            current_bottom_transition_region = []
            current_left_transition_region = []
            current_right_transition_region = []

            transition_region_min_pixels = 10

            top_transition_points = []
            bottom_transition_points = []
            left_transition_points = []
            right_transition_points = []


            # Looking for columns of pixels of only freespace in overlap regions
            for x in range(offset_x, offset_x + subview_list[subview_index].width):

                # Valid top transition regions
                valid_col = True
                for y in range(offset_y, offset_y + row_overlap_pixels):
                    if grid.getpixel((x,y)) == (0,0,0):
                        valid_col = False
                # Add col to transition region
                if valid_col:
                    for y in range(offset_y, offset_y + row_overlap_pixels):
                        grid.putpixel((x, y), (0,255,0)) # Green
                        current_top_transition_region.append((x,y))
                # Otherwise if we have previously added some cols, finish up a region and add to set
                else:
                    if len(current_top_transition_region) > col_overlap_pixels * transition_region_min_pixels:
                        top_transition_regions.append(current_top_transition_region)
                        current_top_transition_region = []


                # Valid bottom transition regions
                valid_col = True
                for y in range(offset_y + subview_list[subview_index].height - row_overlap_pixels, offset_y + subview_list[subview_index].height):
                    if grid.getpixel((x,y)) == (0,0,0):
                        valid_col = False

                if valid_col:
                    for y in range(offset_y + subview_list[subview_index].height - row_overlap_pixels, offset_y + subview_list[subview_index].height):
                        grid.putpixel((x, y), (0,255,0)) # Green
                        current_bottom_transition_region.append((x,y))
                # Otherwise if we have previously added some cols, finish up a region and add to set
                else:
                    if len(current_bottom_transition_region) > col_overlap_pixels * transition_region_min_pixels:
                        bottom_transition_regions.append(current_bottom_transition_region)
                        current_bottom_transition_region = []



            # Looking for rows of pixels of only freespace in overlap regions
            for y in range(offset_y, offset_y + subview_list[subview_index].height):

                # Valid left transition regions
                valid_row = True
                for x in range(offset_x, offset_x + col_overlap_pixels):
                    if grid.getpixel((x,y)) == (0,0,0):
                        valid_row = False

                if valid_row:
                    for x in range(offset_x, offset_x + col_overlap_pixels):
                        grid.putpixel((x, y), (0,255,0)) # Green
                        current_left_transition_region.append((x,y))
                # Otherwise if we have previously added some rows, finish up a region and add to set
                else:
                    if len(current_left_transition_region) > row_overlap_pixels * transition_region_min_pixels:
                        left_transition_regions.append(current_left_transition_region)
                        current_left_transition_region = []


                # Valid right transition regions
                valid_row = True
                for x in range(offset_x + subview_list[subview_index].width - col_overlap_pixels, offset_x + subview_list[subview_index].width):
                    if grid.getpixel((x,y)) == (0,0,0):
                        valid_row = False
                if valid_row:
                    for x in range(offset_x + subview_list[subview_index].width - col_overlap_pixels, offset_x + subview_list[subview_index].width):
                        grid.putpixel((x, y), (0,255,0)) # Green
                        current_right_transition_region.append((x,y))
                # Otherwise if we have previously added some rows, finish up a region and add to set
                else:
                    if len(current_right_transition_region) > row_overlap_pixels * transition_region_min_pixels:
                        right_transition_regions.append(current_right_transition_region)
                        current_right_transition_region = []


            # # Summary of transition regions
            # print('Subview ' + str(subview_index) + ':')
            # print('\t# of valid top transition regions: ' + str(len(top_transition_regions)))
            # print('\t# of valid bottom transition regions: ' + str(len(bottom_transition_regions)))
            # print('\t# of valid left transition regions: ' + str(len(left_transition_regions)))
            # print('\t# of valid right transition regions: ' + str(len(right_transition_regions)))

            # Save transition regions for each camera or subview
            camera_transition_regions[subview_index].append(top_transition_regions)
            camera_transition_regions[subview_index].append(bottom_transition_regions)
            camera_transition_regions[subview_index].append(left_transition_regions)
            camera_transition_regions[subview_index].append(right_transition_regions)




            # Mark the centroid of each transition region - this is our transition point
            transition_point_marking_radius = 3
            for region in top_transition_regions:
                centroid = FindCentroid(region)
                top_transition_points.append(centroid)

                # Draw centroid marking
                centroid = (int(centroid[0]), int(centroid[1]))
                for square_x in range(centroid[0]-transition_point_marking_radius, centroid[0]+transition_point_marking_radius+1):
                    for square_y in range(centroid[1]-transition_point_marking_radius, centroid[1]+transition_point_marking_radius+1):
                        grid.putpixel((square_x, square_y), (0,0,255)) # Blue

            for region in bottom_transition_regions:
                centroid = FindCentroid(region)
                bottom_transition_points.append(centroid)

                # Draw centroid marking
                centroid = (int(centroid[0]), int(centroid[1]))
                for square_x in range(centroid[0]-transition_point_marking_radius, centroid[0]+transition_point_marking_radius+1):
                    for square_y in range(centroid[1]-transition_point_marking_radius, centroid[1]+transition_point_marking_radius+1):
                        grid.putpixel((square_x, square_y), (0,0,255)) # Blue

            for region in left_transition_regions:
                centroid = FindCentroid(region)
                left_transition_points.append(centroid)

                # Draw centroid marking
                centroid = (int(centroid[0]), int(centroid[1]))
                for square_x in range(centroid[0]-transition_point_marking_radius, centroid[0]+transition_point_marking_radius+1):
                    for square_y in range(centroid[1]-transition_point_marking_radius, centroid[1]+transition_point_marking_radius+1):
                        grid.putpixel((square_x, square_y), (0,0,255)) # Blue

            for region in right_transition_regions:
                centroid = FindCentroid(region)
                right_transition_points.append(centroid)

                # Draw centroid marking
                centroid = (int(centroid[0]), int(centroid[1]))
                for square_x in range(centroid[0]-transition_point_marking_radius, centroid[0]+transition_point_marking_radius+1):
                    for square_y in range(centroid[1]-transition_point_marking_radius, centroid[1]+transition_point_marking_radius+1):
                        grid.putpixel((square_x, square_y), (0,0,255)) # Blue

            # Save transition points for each camera or subview
            camera_transition_points[subview_index].append(top_transition_points)
            camera_transition_points[subview_index].append(bottom_transition_points)
            camera_transition_points[subview_index].append(left_transition_points)
            camera_transition_points[subview_index].append(right_transition_points)


    # Identify possible transition regions
    #   Looks for any and all free space in the overlap regions of subviews or cameras
    #   TODO - Need to save these regions
    #   Helpful for the future as I will want to trim transition regions of each camera based
    #   on if it intersects a unsuitable transition region for a neighbor
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            for x in range(offset_x, offset_x + subview_list[subview_index].width):
                # Possible top transition regions
                for y in range(offset_y, offset_y + row_overlap_pixels):
                    if grid.getpixel((x,y)) == (255,255,255):
                        grid.putpixel((x, y), (255,255,0)) # Yellow

                # Possible bottom transition regions
                for y in range(offset_y + subview_list[subview_index].height - row_overlap_pixels, offset_y + subview_list[subview_index].height):
                    if grid.getpixel((x,y)) == (255,255,255):
                        grid.putpixel((x, y), (255,255,0)) # Yellow

            for y in range(offset_y, offset_y + subview_list[subview_index].height):
                # Possible left transition regions
                for x in range(offset_x, offset_x + col_overlap_pixels):
                    if grid.getpixel((x,y)) == (255,255,255):
                        grid.putpixel((x, y), (255,255,0)) # Yellow

                # Possible right transition regions
                for x in range(offset_x + subview_list[subview_index].width - col_overlap_pixels, offset_x + subview_list[subview_index].width):
                    if grid.getpixel((x,y)) == (255,255,255):
                        grid.putpixel((x, y), (255,255,0)) # Yellow


    # Add borders for each subview such that overlaps can be seen
    border_color = (200,0,0)
    for row in range(0, subview_rows):
        for col in range(0, subview_cols):
            subview_index = row * subview_cols + col

            offset_x = col * (subview_width - col_overlap_pixels)
            offset_y = row * (subview_height - row_overlap_pixels)

            # Add borders
            border_width = 2
            for x in range(offset_x, offset_x + subview_list[subview_index].width):
                # Top border for each subview
                for y in range(offset_y, offset_y + border_width):
                    grid.putpixel((x, y), border_color)
                # Bottom border for each subview
                for y in range(offset_y + subview_list[subview_index].height - border_width, offset_y + subview_list[subview_index].height):
                    grid.putpixel((x, y), border_color)

            for y in range(offset_y, offset_y + subview_list[subview_index].height):
                # Left border for each subview
                for x in range(offset_x, offset_x + border_width):
                    grid.putpixel((x, y), border_color)
                # Right border for each subview
                for x in range(offset_x + subview_list[subview_index].width - border_width, offset_x + subview_list[subview_index].width):
                    grid.putpixel((x, y), border_color)


    # Finish up
    grid.save(output_filename)







## MAIN

plt.ion()




# Vars
camera_count = 9



# Camera specific objects are held in lists and referenced by camera index
camera_ids = []                         # Camera ids, establishes a camera's index in the proceeding lists
camera_images = []                      # Images for each camera
camera_free_space_data = []             # Data points corresponding to the location of free space pixels
camera_occupied_space_data = []         # Data points corresponding to the location of free space pixels
camera_gng_instances = []               # GrowingNeuralGas instances



# Create camera grid data structure
camera_grid_rows = 3
camera_grid_cols = 3

# Camera grid only contains camera IDs
print('Initializing camera grid...')
camera_grid = init_camera_grid(camera_grid_rows, camera_grid_cols)

# Transition regions for each camera
camera_transition_regions = []

# Transition points for each camera
camera_transition_points = []



for row in range(0, camera_grid_rows):
    for col in range(0, camera_grid_cols):
        camera_index = row * camera_grid_cols + col

        camera_grid[row][col] = camera_index
        camera_ids.append(camera_index)

        # Add transition region list and transition point list for each camera
        camera_transition_regions.append([])    # List will be populated with Top, Bottom, Left, Right transition regions
        camera_transition_points.append([])    # List will be populated with Top, Bottom, Left, Right transition regions

print('Camera grid:')
for r in range(0, camera_grid_rows):
    print(camera_grid[r])





# Specify a TestEnvironment
#   Test Environment 1 is a 1880x1400 image popualated with some obstacles. The enviornment or image was
#   generated such that it can be split into subviews, with a standard 640x480 resolution, of a 3x3 grid
#   that includes an overlap of 20 pixels on rows and cols. Indexed left to right, top to bottom
print('Populating camera_images list with TestEnvironment1 subviews...')
test_environment_directory = 'Images/TestEnvironments/Environment1'
subview_directory = test_environment_directory + '/3x3'

subview_filenames_list = list(sorted(glob.glob(subview_directory + '/*_subview_*')))
print('\t' + str(len(subview_filenames_list)) + ' subviews found in ' + subview_directory)
for i in range(0, len(subview_filenames_list)):
    print('\t' + subview_filenames_list[i])



# Stitch the images into a coherent environment view and determine transition regions
print('Stitching subviews...')

environment_transition_regions_filename = subview_directory + '/Environment_Transition_Regions.png'
FindTransitionRegions(camera_transition_regions, camera_transition_points, environment_transition_regions_filename, subview_directory, 3, 3, 20, 20)

# Summary of transition regions
for subview_index in range(0, len(camera_ids)):
    print('Subview ' + str(subview_index) + ':')
    print('\t# of valid top transition regions: '       + str(len(camera_transition_regions[subview_index][0])))
    print('\t# of valid bottom transition regions: '    + str(len(camera_transition_regions[subview_index][1])))
    print('\t# of valid left transition regions: '      + str(len(camera_transition_regions[subview_index][2])))
    print('\t# of valid right transition regions: '     + str(len(camera_transition_regions[subview_index][3])))
    print('\t# of valid top transition points: '        + str(len(camera_transition_points[subview_index][0])))
    print('\t# of valid bottom transition points: '     + str(len(camera_transition_points[subview_index][1])))
    print('\t# of valid left transition points: '       + str(len(camera_transition_points[subview_index][2])))
    print('\t# of valid right transition points: '      + str(len(camera_transition_points[subview_index][3])))





# Open the subviews and add to the camera_images list
#   MATPLOTLIB IMAGES
for camera_index in range(0, camera_count):
    camera_images.append(io.imread(subview_filenames_list[camera_index]))




# Preprocess camera images
for camera_index in range(0, camera_count):
    # Ensure grayscale
    greyscale = color.rgb2grey(camera_images[camera_index])

    # The above statement should work but only thresholding gives me true binary
    threshold_adjustment = 0.1
    thresh = threshold_otsu(greyscale) + threshold_adjustment
    binary = greyscale > thresh

    # Scale image down
    scaling_factor = 0.25
    scaled_img = rescale(binary, scaling_factor, anti_aliasing=False)

    camera_images[camera_index] = scaled_img

    # plt.imshow(camera_grid[r][c], cmap='gray')
    # plt.pause(1)
    # plt.show()








# Prepare for GNG
for camera_index in range(0, camera_count):

    # Separate images as arrays of free space pixels and occupied space pixels
    free_space_data, occupied_space_data = image_to_data(camera_images[camera_index])

    # print('Number of free space data points: {:,}'.format(len(free_space_data)))

    # Save data points
    camera_free_space_data.append(free_space_data)
    camera_occupied_space_data.append(occupied_space_data)





# Neupy func for using reproducible seeds, makes results reproducible
utils.reproducible()

for camera_index in range(0, camera_count):
    camera_gng_instances.append(create_gng(max_nodes=1000))

# Saving images
waypoint_image_directory = test_environment_directory + '/3x3/WaypointGeneration'
if not os.path.exists(waypoint_image_directory):
    os.makedirs(waypoint_image_directory)

# Training
for epoch in range(20):

    print('Training epoch: ' + str(epoch))

    # New folder for each epoch
    current_epoch_directory = waypoint_image_directory + '/Epoch_' + str(epoch)
    if not os.path.exists(current_epoch_directory):
        os.makedirs(current_epoch_directory)

    for camera_index in range(0, camera_count):

        # Generate a subview image that shows the waypoint graph in the free space
        fig = plt.figure(figsize=(6.4, 4.8))
        plt.xticks([], [])
        plt.yticks([], [])

        # Plot occupied space for reference
        plt.scatter(*np.array(camera_occupied_space_data[camera_index]).T , s=25, alpha=1, color='black');

        camera_gng_instances[camera_index].train(camera_free_space_data[camera_index], epochs=1)
        print('Camera ' + str(camera_index) + ' gng status:')
        gng_status(camera_gng_instances[camera_index])

        # Draw graph of gng instance in free space
        for node_1, node_2 in camera_gng_instances[camera_index].graph.edges:
            weights = np.concatenate([node_1.weight, node_2.weight])
            line, = plt.plot(*weights.T, color='darkgray')
            plt.setp(line, linewidth=0.5, color='darkgray')

        # Save this figure
        waypoint_image_filename = current_epoch_directory + '/subview_' + str(camera_index) + '.png'

        # Credit to https://stackoverflow.com/a/53516034/11445242
        #   I spent much too much time on making a decent image from matplotlib figs
        plt.subplots_adjust(0,0,1,1,0,0)
        for ax in fig.axes:
            ax.axis('off')
            ax.margins(0,0)
            ax.xaxis.set_major_locator(plt.NullLocator())
            ax.yaxis.set_major_locator(plt.NullLocator())
        fig.savefig(waypoint_image_filename, pad_inches = 0, bbox_inches='tight')

        plt.close(fig)
        # plt.pause(0.0001)
        # plt.show()





    # At the end of each epoch, stitch all the produced images for a nice visualization
    print('Stitching gng graphs for epoch ' + str(epoch))

    environment_waypoint_image_directory = waypoint_image_directory + '/StitchedImages'
    if not os.path.exists(environment_waypoint_image_directory):
        os.makedirs(environment_waypoint_image_directory)

    environment_waypoint_image_filename = environment_waypoint_image_directory + '/Epoch_' + '{0:0=2d}'.format(epoch) + '_environment.png'
    stitchGNGImagesToGrid(camera_transition_regions, camera_transition_points, environment_waypoint_image_filename, current_epoch_directory, 3, 3, 20, 20)





# for r in range(0, camera_grid_rows):
#     for c in range(0, camera_grid_cols):
#
#         # New instance of gng for each subview
#         camera_grid_gng_instances[r][c] = create_gng(max_nodes=1000)
#
#
# # Train
# for epoch in range(20):
#     print('Training epoch: ' + str(epoch))
#
#     for r in range(0, camera_grid_rows):
#         for c in range(0, camera_grid_cols):
#
#             camera_index = r * camera_grid_cols + c
#             print('\tCamera index: ' + str(camera_index))
#
#
#             camera_grid_gng_instances[r][c].train(camera_grid_free_space_data[r][c], epochs=1)
#             gng_status(camera_grid_gng_instances[r][c])




# # Create a Grwoing Nerual Gas network
# gng = create_gng(max_nodes=1000)
#
#
# for epoch in range(20):
#     gng.train(free_space_data, epochs=1)
#     gng_status(gng)
#
#     # Plot images after each iteration in order to see training progress
#     plt.figure(figsize=(6.4, 4.8))
#     plt.scatter(*np.array(occupied_space_data).T , s=1, alpha=1, color=next(colors));
#
#     draw_image(gng.graph)




while 1:
    continue








#
# # Get and preprocess imaages
# input_image = io.imread('Images/TestEnvironments/Environment1/Environment.png')
#
#
#
#
#
# # Get image
# # input_image = io.imread('test_binary_img.jpg')
# # input_image = io.imread('ExampleEnvironment.png')
# input_image = io.imread('Images/TestEnvironments/Environment1/Environment.png')
# print('Original image size: ' + str(input_image.shape))
#
# # Ensure binary
# input_image_grey = color.rgb2grey(input_image)
# print('Greyscsale image size: ' + str(input_image_grey.shape))
#
# # # Show original and resulting processed images
# # plt.figure(figsize=(16, 6))
# #
# # plt.subplot(141)
# # plt.title('Original image')
# # plt.imshow(input_image)
# #
# # plt.subplot(142)
# # plt.title('Greyscaled image')
# # plt.imshow(input_image_grey, cmap='gray')
#
# # Increase threshold in order to add more
# # details to the binarized image
# threshold_adjustment = 0.1
# thresh = threshold_otsu(input_image_grey) + threshold_adjustment
# input_image_binary = input_image_grey > thresh
#
# # plt.subplot(143)
# # plt.title('Binarized image')
# # plt.imshow(input_image_binary, cmap='gray');
# #
# # # Rescale image for faster processing
# # scaling_factor = 0.25
# # input_image_binary_rescaled = rescale(input_image_binary, scaling_factor, anti_aliasing=False)
# # plt.subplot(144)
# # plt.title('Rescaled image (' + str(scaling_factor) + ')')
# # plt.imshow(input_image_binary_rescaled, cmap='gray');
# #
# # plt.pause(0.0001)
# # plt.show()
#
# # Split environment to 3x3 grid
# M = 480
# N = 640
# tiles = [input_image_grey[x:x+M,y:y+N] for x in range(0,input_image_grey.shape[0],M) for y in range(0,input_image_grey.shape[1],N)]
#
# plt.figure(figsize=(6.4, 4.8))
# plt.subplot(331)
# plt.imshow(tiles[0], cmap='gray');
# #plt.axis('off')
# plt.subplot(332)
# plt.imshow(tiles[1], cmap='gray');
# #plt.axis('off')
# plt.subplot(333)
# plt.imshow(tiles[2], cmap='gray');
# # plt.axis('off')
# plt.subplot(334)
# plt.imshow(tiles[3], cmap='gray');
# #plt.axis('off')
# plt.subplot(335)
# plt.imshow(tiles[4], cmap='gray');
# #plt.axis('off')
# plt.subplot(336)
# plt.imshow(tiles[5], cmap='gray');
# #plt.axis('off')
# plt.subplot(337)
# plt.imshow(tiles[6], cmap='gray');
# #plt.axis('off')
# plt.subplot(338)
# plt.imshow(tiles[7], cmap='gray');
# #plt.axis('off')
# plt.subplot(339)
# plt.imshow(tiles[8], cmap='gray');
# #plt.axis('off')
#
# plt.pause(0.0001)
# plt.show()
#
#
# while 1:
#     continue
#
#
#
# # Get image as an array of free space pixels
# free_space_data, occupied_space_data = image_to_data(tiles[0])
#
#
# # Scaling factor allows us to reduce distance between data points
# scale_factor = 0.001
# free_space_data = scale_factor * np.array(free_space_data)
# occupied_space_data = scale_factor * np.array(occupied_space_data)
# print('Number of free space data points: {:,}'.format(len(free_space_data)))
#
# # Show scatter plot of whitespace pixels
# plt.figure(figsize=(9, 8))
# colors = itertools.cycle(['r', 'b', 'g'])
# plt.scatter(*np.array(occupied_space_data).T / scale_factor , s=1, alpha=1, color=next(colors));
# plt.scatter(*np.array(free_space_data).T / scale_factor , s=1, alpha=1, color=next(colors));
#
# plt.pause(0.0001)
# plt.show()
#
#
# # Neupy func for using reproducible seeds, makes results reproducible
# utils.reproducible()
#
# # Create a Grwoing Nerual Gas network
# gng = create_gng(max_nodes=1000)
#
#
# for epoch in range(20):
#     gng.train(free_space_data, epochs=1)
#     gng_status(gng)
#
#     # Plot images after each iteration in order to see training progress
#     plt.figure(figsize=(6.4, 4.8))
#     plt.scatter(*np.array(occupied_space_data).T , s=1, alpha=1, color=next(colors));
#
#     draw_image(gng.graph)
