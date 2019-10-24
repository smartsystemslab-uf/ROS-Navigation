import argparse
import glob
import sys

from PIL import Image



# Arguments
parser = argparse.ArgumentParser(description='Convert a collection of images or subviews into a single image or grid.')
parser.add_argument('subview_directory', metavar='subview_directory', nargs='+', help='Directory containing images to be stitched. Looks for images with filenames ending with \'_subview_<index>\'. Will iterate through indexes and stitch according to below args. Output image, \'grid.png\', will be placed in the same directory.')
parser.add_argument('rows', metavar='rows', nargs='+', help='Number of rows in stitched grid')
parser.add_argument('cols', metavar='cols', nargs='+', help='Number of rows in stitched grid')
parser.add_argument('row_overlap', metavar='row_coverlap', nargs='+', help='Number of pixels each row overlaps neighboring rows in stitched grid')
parser.add_argument('col_overlap', metavar='col_overlap', nargs='+', help='Number of pixels each column overlaps neighboring columns in stitched grid')

# Ensure we get appropriate args, otherwise print a help output
args = parser.parse_args()


subview_directory = sys.argv[1]

subview_rows = int(sys.argv[2])
subview_cols = int(sys.argv[3])

row_overlap_pixels = int(sys.argv[4])
col_overlap_pixels = int(sys.argv[5])


subview_list = list(map(Image.open, glob.glob(subview_directory + '/*_subview_*')))
print(str(len(subview_list)) + ' subviews found in ' + subview_directory)

for im in subview_list:
    print(im.filename)

# All subview sizes exptected to be the same
subview_width = subview_list[0].size[0]
subview_height = subview_list[0].size[1]


output_width = (subview_cols * subview_width) - (col_overlap_pixels * (subview_cols - 1))
output_height = (subview_rows * subview_height) - (row_overlap_pixels * (subview_rows - 1))

grid = Image.new('L', (output_width, output_height), 'black')


# Do the splits
for row in range(0, subview_rows):
    for col in range(0, subview_cols):
        subview_index = row * subview_cols + col

        for subview in list(subview_list):

            if str(subview_index) in subview.filename[subview.filename.index('_subview_'):]:
                offset_x = col * (subview_width - col_overlap_pixels)
                offset_y = row * (subview_height - row_overlap_pixels)
                grid.paste(subview, (offset_x, offset_y))
                break


grid.save(subview_directory + '/grid.png')
