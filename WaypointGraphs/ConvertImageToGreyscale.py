from skimage import color, io
import sys

input_filename = sys.argv[1]
print('Converting ' + input_filename)

# Get image
input_image = io.imread(input_filename)
print('Original image size: ' + str(input_image.shape))

# Ensure binary
input_image_grey = color.rgb2grey(input_image)
print('Greyscsale image size: ' + str(input_image_grey.shape))

io.imsave(('greyscale_' + input_filename), input_image_grey)
