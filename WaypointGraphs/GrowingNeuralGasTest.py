# import skimage
from skimage import io, data, img_as_float
from skimage.filters import threshold_otsu, threshold_local, gaussian
from skimage import color

import numpy as np

from neupy import algorithms, utils

import matplotlib.pyplot as plt

import os
import sys


CURRENT_DIR = os.path.abspath(os.path.dirname(__name__))
LIBRARY_DIR = os.path.join(CURRENT_DIR, '..', '..')

def saveas(name):
    image_name = '{}.png'.format(name)
    image_path = os.path.join(LIBRARY_DIR, 'site', '2018', '03', '26', 'images', image_name)
    plt.savefig(image_path, facecolor='#f8fafb', bbox_inches='tight')



# astro = img_as_float(data.astronaut())
astro = io.imread('test_binary_img.jpg')

# astro = astro[30:180, 150:300]
plt.imshow(astro);


plt.figure(figsize=(16, 6))

plt.subplot(141)
plt.title("Original image")
plt.imshow(astro)

astro_grey = color.rgb2grey(astro)

plt.subplot(142)
plt.title("Greyscaled image")
plt.imshow(astro_grey, cmap='gray')

# We added gaussian smoothing in order to reduce
# number of very small regions
astro_grey = gaussian(astro_grey, sigma=0.6)

plt.subplot(143)
plt.title("Greyscaled image \nwith gaussian smoothing")
plt.imshow(astro_grey, cmap='gray')

# Increase threshold in order to add more
# details to the binarized image
threshold_adjustment = 0.1
thresh = threshold_otsu(astro_grey) + threshold_adjustment
binary_astro = astro_grey < thresh

plt.subplot(144)
plt.title("Binarized image")
plt.imshow(astro_grey > thresh, cmap='gray');




def image_to_data(img):
    data = []
    for (x, y), value in np.ndenumerate(img):
        if value == 0:
            data.append([y, -x])
    return data

# Convert each black pixel to a separate data point
# We will use these data points in order to learn
# topological structure of the image
data = image_to_data(binary_astro)

# Scaling factor allows us to reduce distance between data points
scale_factor = 0.001
data = scale_factor * np.array(data)
print("Number of data points: {:,}".format(len(data)))

plt.figure(figsize=(6, 8))
plt.scatter(*np.array(data).T / scale_factor, s=5, alpha=1);


# np.savetxt('Point.pts', np.array(data))


plt.ion()
plt.show()


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

def create_gng(max_nodes, step=0.2, n_start_nodes=100, max_edge_age=50):
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



utils.reproducible()
gng = create_gng(max_nodes=100)

for epoch in range(20):
    gng.train(data, epochs=1)

    # Plot images after each iteration in order to see training progress
    plt.figure(figsize=(5.5, 6))
    draw_image(gng.graph)
