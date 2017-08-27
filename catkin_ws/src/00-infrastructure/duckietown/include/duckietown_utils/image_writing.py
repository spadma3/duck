from duckietown_utils.jpg import jpg_from_image_cv
from duckietown_utils.file_utils import write_data_to_file
import numpy as np

def write_image_as_jpg(image, filename):
    if not isinstance(image, np.ndarray):
        # XXX
        pass
    jpg = jpg_from_image_cv(image)
    write_data_to_file(jpg, filename)
    