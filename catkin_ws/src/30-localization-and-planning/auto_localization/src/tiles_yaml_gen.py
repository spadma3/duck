import yaml

tiles = {
        'c': "curve_left",
        's': "straight",
        '3w': "3way_left",
        '4w': "4way",
        'g': "grass"}
direction = {
        'w': "/W",
        'e': "/E",
        'n': "/N",
        's': "/S",
}

key_enter = ''
this_tile = ''

print "Please enter the siye of the map"
row = input("Row: ")
col = input("Col: ")

map_tiles = [[]]
row_i = 0
col_i = 0

######################
while key_enter != 'q':

    print("Enter next tile, press q to quit")
    key_enter = raw_input("Next tile is?  ")
    key_enter = str(key_enter)

    if key_enter[:-1] in tiles:
        this_tile = tiles[key_enter[:-1]]
    elif key_enter in tiles:
        this_tile = tiles[key_enter]
    elif key_enter == 'q':
        exit()
    else:
        print "You are only allow to enter: c, s, 3w, 4w, g. With direction w, e, n, s"
        continue

    if key_enter == '4w' or key_enter == 'g':
        pass
    else:
        if key_enter[-1] in direction:
            this_tile += direction[key_enter[-1]]
        else:
            print "Please add direction w, e, n, s"
            continue

    map_tiles[row_i].append(this_tile)
    col_i +=1
    if col_i == col:
        map_tiles.append(list())
        row_i += 1
        col_i = 0
    if row_i == row:
        break

    print map_tiles

####################

with open("gen.yaml", 'w') as outfile:
    yaml.dump(map_tiles, outfile, default_flow_style=False)
