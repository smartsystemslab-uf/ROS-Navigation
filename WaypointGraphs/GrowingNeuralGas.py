import os
import sys

import itertools
import glob

import matplotlib.pyplot as plt
import numpy as np

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
        verbose=True,

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
    print('Current node count: ' + str(gng.graph.n_nodes))
    print('Total training updates: ' + str(gng.n_updates_made))

def init_camera_grid(camera_grid_rows, camera_grid_cols):
    rows = [i for i in range(camera_grid_rows)]
    columns = [i for i in range(camera_grid_cols)]
    return {r : {c:None for c in columns} for r in rows}




plt.ion()


# Create camera grid data structure
camera_grid_rows = 3
camera_grid_cols = 3

print('Initializing camera grid...')
camera_grid = init_camera_grid(camera_grid_rows, camera_grid_cols)

# Data structures used later on similar layouts
camera_grid_free_space_data = camera_grid
camera_grid_occupied_space_data = camera_grid

camera_grid_gng_instances = camera_grid

print('Camera grid:')
for r in range(0, camera_grid_rows):
    print(camera_grid[r])



# Populate the camera_grid with a TestEnvironment
#   Test Environment 1 is a 1880x1400 image popualated with some obstacles. The enviornment or image was
#   generated such that it can be split into subviews, with a standard 640x480 resolution, of a 3x3 grid
#   that includes an overlap of 20 pixels on rows and cols.
print('Populating camera grid with TestEnvironment1 subviews...')
test_environment_directory = 'Images/TestEnvironments/Environment1'
subview_directory = test_environment_directory + '/3x3'

subview_filenames_list = list(sorted(glob.glob(subview_directory + '/*_subview_*')))
print(str(len(subview_filenames_list)) + ' subviews found in ' + subview_directory)
for i in range(0, len(subview_filenames_list)):
    print('\t' + subview_filenames_list[i])

# Open the subviews and add to the camera grid
#   Subview index and camera index should be identical
for r in range(0, camera_grid_rows):
    for c in range(0, camera_grid_cols):
        camera_index = r * camera_grid_cols + c
        camera_grid[r][c] = io.imread(subview_filenames_list[camera_index])



# Preprocess camera_grid images
for r in range(0, camera_grid_rows):
    for c in range(0, camera_grid_cols):

        # Ensure grayscale
        greyscale = color.rgb2grey(camera_grid[r][c])

        # The above statement should work but only thresholding gives me true binary
        threshold_adjustment = 0.1
        thresh = threshold_otsu(greyscale) + threshold_adjustment
        binary = greyscale > thresh

        # Scale image down
        scaling_factor = 0.25
        scaled_img = rescale(binary, scaling_factor, anti_aliasing=False)

        camera_grid[r][c] = scaled_img

        # plt.imshow(camera_grid[r][c], cmap='gray')
        # plt.pause(1)
        # plt.show()






# Prepare for GNG
for r in range(0, camera_grid_rows):
    for c in range(0, camera_grid_cols):

        # Separate images as arrays of free space pixels and occupied space pixels
        free_space_data, occupied_space_data = image_to_data(camera_grid[r][c])


        # Scaling factor allows us to reduce distance between data points
        scale_factor = 0.001
        free_space_data = scale_factor * np.array(free_space_data)
        occupied_space_data = scale_factor * np.array(occupied_space_data)
        print('Number of free space data points: {:,}'.format(len(free_space_data)))





        # Show scatter plot of free and occupied space
        plt.figure(figsize=(9, 8))
        colors = itertools.cycle(['r', 'b'])
        plt.scatter(*np.array(occupied_space_data).T/scale_factor, s=1, alpha=1, color=next(colors));   # RED
        plt.scatter(*np.array(free_space_data).T/scale_factor, s=1, alpha=1, color=next(colors));       # BLUE



        # Save free space data
        # print(free_space_data)
        camera_grid_free_space_data[r][c] = free_space_data
        # print(camera_grid_free_space_data[r][c])

        camera_grid_occupied_space_data[r][c] = occupied_space_data





        # plt.pause(5)
        # plt.show()


# Neupy func for using reproducible seeds, makes results reproducible
utils.reproducible()

for r in range(0, camera_grid_rows):
    for c in range(0, camera_grid_cols):

        # New instance of gng for each subview
        camera_grid_gng_instances[r][c] = create_gng(max_nodes=1000)


# Train
for epoch in range(20):
    print('Training epoch: ' + str(epoch))

    for r in range(0, camera_grid_rows):
        for c in range(0, camera_grid_cols):

            camera_index = r * camera_grid_cols + c
            print('\tCamera index: ' + str(camera_index))


            camera_grid_gng_instances[r][c].train(camera_grid_free_space_data[r][c], epochs=1)
            gng_status(camera_grid_gng_instances[r][c])




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
