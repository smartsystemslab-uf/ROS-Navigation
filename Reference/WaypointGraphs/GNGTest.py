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






input = io.imread('Images/Circle.jpg')




greyscale = color.rgb2grey(input)

# The above statement should work but only thresholding gives me true binary
threshold_adjustment = 0.1
thresh = threshold_otsu(greyscale) + threshold_adjustment
binary = greyscale > thresh

# Scale image down
scaling_factor = 0.25
scaled_img = rescale(binary, scaling_factor, anti_aliasing=False)

operable_image = scaled_img

# plt.imshow(camera_grid[r][c], cmap='gray')
# plt.pause(1)
# plt.show()


# Separate images as arrays of free space pixels and occupied space pixels
free_space_data, occupied_space_data = image_to_data(operable_image)

# print('Number of free space data points: {:,}'.format(len(free_space_data)))


# Neupy func for using reproducible seeds, makes results reproducible
utils.reproducible()

gng = create_gng(max_nodes=1000)
print('Gng initial status:')
gng_status(gng)

# Training
for epoch in range(200):

    print('Training epoch: ' + str(epoch))

    # Generate a subview image that shows the waypoint graph in the free space
    fig = plt.figure(figsize=(12.8, 9.6))
    plt.xticks([], [])
    plt.yticks([], [])

    # Plot occupied space for reference
    plt.scatter(*np.array(occupied_space_data).T , s=25, alpha=1, color='white');

    gng.train(free_space_data, epochs=1)
    print('Gng status:')
    gng_status(gng)

    # Draw graph of gng instance in free space
    for node_1, node_2 in gng.graph.edges:
        weights = np.concatenate([node_1.weight, node_2.weight])
        line, = plt.plot(*weights.T, color='black')
        plt.setp(line, linewidth=0.5, color='black')

    # Save this figure
    waypoint_image_filename = 'Epoch_' + str(epoch) + '_waypoints.png'

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
