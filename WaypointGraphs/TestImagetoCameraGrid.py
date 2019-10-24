import matplotlib.pyplot as plt
from skimage import color, data, io


# Get image
# input_image = io.imread('test_binary_img.jpg')
# input_image = io.imread('ExampleEnvironment.png')
input_image = io.imread('3x3CameraGridEnvironment.png')
print('Original image size: ' + str(input_image.shape))

# Ensure binary
input_image_grey = color.rgb2grey(input_image)
print('Greyscsale image size: ' + str(input_image_grey.shape))


# 3x3 grid
orig_height = input_image_grey.shape[0]
orig_width = input_image_grey.shape[1]

subview_rows = 3
subview_cols = 3

# Overlaps
overlap_row_pixels = 20
overlap_col_pixels = 20


subview_height = (orig_height + (overlap_row_pixels * (subview_rows - 1))) / subview_rows
subview_width = (orig_width + (overlap_col_pixels * (subview_cols - 1))) / subview_cols

print('Subview sizes with 3x3 grid: (' + str(subview_height) + ',' + str(subview_width) + ')')


subviews = [input_image_grey[x:x+int(subview_height),y:y+int(subview_width)] for x in range(0,orig_height,int(subview_height)-overlap_row_pixels) for y in range(0,orig_width,int(subview_width) - overlap_col_pixels)]


# plt.ion()
plt.figure(figsize=(6.4, 4.8))
plt.subplot(331)
plt.imshow(subviews[0], cmap='gray');
print('Subview 0: ' + str(len(subviews[0])) + ',' + str(len(subviews[0][0])))
#plt.axis('off')
plt.subplot(332)
plt.imshow(subviews[1], cmap='gray');
print('Subview 1: ' + str(len(subviews[1])) + ',' + str(len(subviews[1][0])))
#plt.axis('off')
plt.subplot(333)
plt.imshow(subviews[2], cmap='gray');
print('Subview 2: ' + str(len(subviews[2])) + ',' + str(len(subviews[2][0])))
# plt.axis('off')
plt.subplot(334)
plt.imshow(subviews[3], cmap='gray');
print('Subview 3: ' + str(len(subviews[3])) + ',' + str(len(subviews[3][0])))
#plt.axis('off')
plt.subplot(335)
plt.imshow(subviews[4], cmap='gray');
print('Subview 4: ' + str(len(subviews[4])) + ',' + str(len(subviews[4][0])))
#plt.axis('off')
plt.subplot(336)
plt.imshow(subviews[5], cmap='gray');
print('Subview 5: ' + str(len(subviews[5])) + ',' + str(len(subviews[5][0])))
#plt.axis('off')
plt.subplot(337)
plt.imshow(subviews[6], cmap='gray');
print('Subview 6: ' + str(len(subviews[6])) + ',' + str(len(subviews[6][0])))
#plt.axis('off')
plt.subplot(338)
plt.imshow(subviews[7], cmap='gray');
print('Subview 7: ' + str(len(subviews[7])) + ',' + str(len(subviews[7][0])))
#plt.axis('off')
plt.subplot(339)
plt.imshow(subviews[8], cmap='gray');
print('Subview 8: ' + str(len(subviews[8])) + ',' + str(len(subviews[8][0])))

#plt.axis('off')

plt.pause(0.0001)
plt.show()
