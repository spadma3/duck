class Transformer():
    def __init__(self,tile_size,number_of_tiles_y):
        self.tile_size = float(tile_size) # to prevent python from rounding prematurely
        self.tiles_y = number_of_tiles_y

    # this maps a zero-based tile location to a tile-midpoint in pixel coordinates
    def map_to_image(self, point_on_map):
        return (int((point_on_map[0] + 0.5 )* self.tile_size),
                int((self.tiles_y - point_on_map[1] - 0.5) * self.tile_size))

    # this maps a pixel point to the zero-based tile location the pixel is in
    def image_to_map(self, point_on_image):
        y_temp = self.tiles_y - float((point_on_image[1] + 1) / self.tile_size)
        x_temp = int(point_on_image[0] / self.tile_size)
        return (x_temp,int(y_temp))
