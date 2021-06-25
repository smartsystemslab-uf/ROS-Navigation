import os
import sys
import time
import shutil

import itertools
import glob

import matplotlib.pyplot as plt
import numpy as np
##import cv2

from PIL import Image

from neupy import algorithms, utils
from skimage import color, data, io, img_as_float,img_as_ubyte
from skimage.filters import threshold_otsu, threshold_local, gaussian
from skimage.transform import rescale, resize, downscale_local_mean

import picamera
from time import sleep


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

        n_iter_before_neuron_added=500,
        after_split_error_decay_rate=0.5,
        error_decay_rate=0.995,
        min_distance_for_update=0.01,
    )

def gng_status(gng):
    print('\tCurrent node count: ' + str(gng.graph.n_nodes))
    print('\tTotal training updates: ' + str(gng.n_updates_made))


directory = 'Camera1'
if os.path.isdir(directory):
    shutil.rmtree(directory)

os.mkdir(directory)
    


camera_ids = []                         # Camera ids, establishes a camera's index in the proceeding lists
camera_images = []                      # Images for each camera
camera_free_space_data = []             # Data points corresponding to the location of free space pixels
camera_occupied_space_data = []         # Data points corresponding to the location of free space pixels
camera_gng_instances = []               # GrowingNeuralGas instances

camera = picamera.PiCamera()
camera.start_preview()
sleep(5)
camera.capture('/home/pi/Desktop/Robotics_Image_Processing/image.png')
camera.stop_preview()

input = io.imread('/home/pi/Desktop/Robotics_Image_Processing/image.png')
#TestEnvironments/Environment1/3x3/Environment_subview_0.png'
#cv2.imwrite('gray.png', input)

greyscale = color.rgb2gray(input)
io.imsave('grayscale.png', img_as_ubyte(greyscale))



# The above statement should work but only thresholding gives me true binary
threshold_adjustment = 0.1
thresh = threshold_otsu(greyscale) + threshold_adjustment
print('Thresh is: ', thresh)
binary = (greyscale > thresh).astype(float)
io.imsave('binary.png', img_as_ubyte(binary))


# Scale image down
scaling_factor = 0.25
scaled_img = rescale(binary, scaling_factor, anti_aliasing=False)

operable_image = scaled_img

##operable_image.dtype='uint8'
# plt.imshow(camera_grid[r][c], cmap='gray')
# plt.pause(1)
# plt.show()


# Separate images as arrays of free space pixels and occupied space pixels
free_space_data, occupied_space_data = image_to_data(operable_image)

# print('Number of free space data points: {:,}'.format(len(free_space_data)))
camera_free_space_data.append(free_space_data)
camera_occupied_space_data.append(occupied_space_data)

# Neupy func for using reproducible seeds, makes results reproducible
utils.reproducible()

gng = create_gng(max_nodes=1000)
print('Gng initial status:')
gng_status(gng)

# Training
for epoch in range(20):

    print('Training epoch: ' + str(epoch))
    start = time.time()
    # Generate a subview image that shows the waypoint graph in the free space
    fig = plt.figure(figsize=(12.8, 9.6))
    plt.xticks([], [])
    plt.yticks([], [])

    # Plot occupied space for reference
    plt.scatter(*np.array(camera_occupied_space_data).T , s=25, alpha=1, color='black');

    gng.train(free_space_data, epochs=1)
    print('Gng status:')
    gng_status(gng)

    # Draw graph of gng instance in free space
    for node_1, node_2 in gng.graph.edges:
        weights = np.concatenate([node_1.weight, node_2.weight])
        line, = plt.plot(*weights.T, color='black')
        plt.setp(line, linewidth=0.5, color='black')
        
    end = time.time()
    runtime = end - start
    print('\tRuntime of latest training epoch: ' + str(runtime) + ' seconds')
    # Save this figure
    waypoint_image_filename = directory + '/Epoch_' + str(epoch) + '_waypoints.png'

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
