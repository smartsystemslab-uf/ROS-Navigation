import argparse
import sys

from wand.image import Image

# Arguments
parser = argparse.ArgumentParser(description='Convert an image to a set of cropped subviews. Outputs rows*cols number of subviews.')
parser.add_argument('filepath', metavar='filepath', nargs='+', help='Input image filepath. Output images will be placed in the same directory')
parser.add_argument('rows', metavar='rows', nargs='+', help='Number of rows in subview grid')
parser.add_argument('cols', metavar='cols', nargs='+', help='Number of rows in subview grid')
parser.add_argument('row_overlap', metavar='row_coverlap', nargs='+', help='Number of pixels each row overlaps neighboring rows in subview grid')
parser.add_argument('col_overlap', metavar='col_overlap', nargs='+', help='Number of pixels each column overlaps neighboring columns in subview grid')

# Ensure we get appropriate args, otherwise print a help output
args = parser.parse_args()


input_filepath = sys.argv[1]
input_filename = sys.argv[1][:sys.argv[1].index('.')]
input_file_extension = sys.argv[1][sys.argv[1].index('.'):]

subview_rows = int(sys.argv[2])
subview_cols = int(sys.argv[3])

row_overlap_pixels = int(sys.argv[4])
col_overlap_pixels = int(sys.argv[5])


print()
print('Converting image to grid with options:')
print('\t' + str(subview_rows) + 'x' + str(subview_cols) + ' grid (rows, cols)')
print('\t' + str(row_overlap_pixels) + ' overlap pixels in rows')
print('\t' + str(col_overlap_pixels) + ' overlap pixels in cols')


# Get Image
input_image = Image(filename=input_filepath)
print('Original image size: ' + str(input_image.size))


# Calculate subview dimensions
# NOTE: There is a forced casting to int
#   Splitting into subviews expects a clean fit of tiles onto input image
subview_height = int((input_image.height + (row_overlap_pixels * (subview_rows - 1))) / subview_rows)
subview_width = int((input_image.width + (col_overlap_pixels * (subview_cols - 1))) / subview_cols)
print('Subview sizes : (' + str(subview_width) + ',' + str(subview_height) + ')')


# Do the splits
for row in range(0, subview_rows):
    for col in range(0, subview_cols):
        subview_index = row * subview_cols + col

        # Filename
        output_filename = input_filename + '_subview_' + str(subview_index) + input_file_extension

        start_x = col * (subview_width - col_overlap_pixels)
        start_y = row * (subview_height - row_overlap_pixels)

        with input_image[start_x:start_x+subview_width, start_y:start_y+subview_height] as crop:
            crop.save(filename=output_filename)


# Better on the eyes to add some space
print()
