'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
import copy
import math
import random
import argparse
import heapq
from heapq import *
from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
from math import ceil

g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells - 1)
    new_map[random_cell] = 1

  return new_map


def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

  return grid


def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  global g_WORLD_MAP
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  #Variable Instantiation
  source_ij = vertex_index_to_ij(vertex_source)
  dest_ij = vertex_index_to_ij(vertex_dest)
  source_i = source_ij[0]
  source_j = source_ij[1]
  dest_i = dest_ij[0]
  dest_j = dest_ij[1]

  # Ensure we are checking vertex indices that are in bound
  if vertex_source >= len(g_WORLD_MAP) or vertex_source < 0:
    return 1000
  elif vertex_dest >= len(g_WORLD_MAP) or vertex_dest < 0:
    return 1000

  # Ensure we aren't going "off the sides" of the map to create shortcuts by ensuring there's not more than a difference of 1
  if abs(source_i - dest_i) > 1:
    return 1000
  if abs(source_j - dest_j) > 1:
    return 1000

  source_barrier_bool = g_WORLD_MAP[vertex_source]
  dest_barrier_bool = g_WORLD_MAP[vertex_dest]

  #Actual if statements and evualuation
  if(source_barrier_bool != 0 or dest_barrier_bool != 0):
      return 1000
  elif(source_i < 0 or source_i > g_NUM_X_CELLS):
      return 1000
  elif(source_j < 0 or source_j > g_NUM_Y_CELLS):
      return 1000
  else:
      return 1


def run_dijkstra(source_vertex, map):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run

  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  dist = [float('inf')] * g_NUM_X_CELLS * g_NUM_Y_CELLS

  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  Q_cost = []

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS

  # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!
  heappush(Q_cost, (source_vertex, 0))
  dist[source_vertex] = 0
  while len(Q_cost) != 0:
    curInd, curCost = heappop(Q_cost)
    left = curInd - 1
    right = curInd + 1
    top = curInd + g_NUM_X_CELLS
    bottom = curInd - g_NUM_X_CELLS
    neighboors = [left, right, top, bottom]
    for neighboor in neighboors:
        cost = get_travel_cost(curInd, neighboor)
        alt = cost + curCost
        if cost < 1000 and alt < dist[neighboor] and map[neighboor] == 0:
            heappush(Q_cost, (neighboor, cost + curCost))
            prev[neighboor] = curInd
            dist[neighboor] = alt

  # Return results of algorithm run
  return prev


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []

  # TODO: Insert your code here
  final_path.append(dest_vertex)
  previous = prev[dest_vertex]
  flag = True
  while(flag):
      if(previous == -1):
          final_path = []
          flag = False
          return []
      elif(previous == source_vertex):
          final_path.append(previous)
          flag = False
      else:
          final_path.append(previous)
          previous = prev[previous]

  return final_path


def render_map(map_array):
  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  rows = []
  row = []

  # Go through map backward and create string lists of what we need to print each row
  for i in range(len(map_array) - 1, -1, -1):
    # Obstacle
    if map_array[i] == 1:
      row.append('[ ]')
    # Free space
    else:
      row.append(" . ")
    # New Row
    if i % g_NUM_Y_CELLS == 0:
      rows.append(row)
      row = []

  # Print each row backward
  for row in rows:
    for i in range(len(row)-1, -1, -1):
      print(row[i], end='')
    print('\n')
  pass


def part_1():
  global g_WORLD_MAP, g_NUM_X_CELLS, g_NUM_Y_CELLS

  # Create random list of 0's that we will add obstacles to...
  #randIntX = np.random.randint(2, 6)
  #randIntY = np.random.randint(2, 6)
  sizeOfMap = g_NUM_X_CELLS * g_NUM_Y_CELLS

  map = np.zeros(sizeOfMap)

  # TODID: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
  testMap = create_test_map(map)

  # Use render_map to render your initialized obstacle map
  render_map(testMap)

  # TODID: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path

  # Get random starting vertex to test Dijkstra's code, ensure we don't start at an obstacle by using while loop
  randSourceVertexIndex = np.random.randint(0, len(testMap))
  while testMap[randSourceVertexIndex] != 0:
    randSourceVertexIndex = np.random.randint(0, len(testMap))
  randSourceIJIndex = vertex_index_to_ij(randSourceVertexIndex)

  # Get random destination vertex to test Dijkstra's code, ensure we don't start at an obstacle by using while loop
  randGoalVertexIndex = np.random.randint(0, len(testMap))
  while testMap[randGoalVertexIndex] != 0:
    randGoalVertexIndex = np.random.randint(0, len(testMap))
  randGoalIJIndex = vertex_index_to_ij(randGoalVertexIndex)

  # Run Dijkstra's on our randomly generated map and our starting source node
  prevArray = run_dijkstra(randSourceVertexIndex,testMap)

  # Reconstruct the path from our random source to our random destination
  # Path takes in prevArray from dijkstra's, takes in vertex indices, returns list of vertex indices
  path = reconstruct_path(prevArray, randSourceVertexIndex, randGoalVertexIndex)

  '''
    TODID-
      Display the final path in the following format:
      Source: (0,0)
      Goal: (3,1)
      0 -> 1 -> 2 -> 6 -> 7
    '''
  print(f"Source: {randSourceIJIndex}")
  print(f"Goal: {randGoalIJIndex}")

  if len(path) == 0 or path is None:
    print('No Path Found!')
  else:
    # Path is returned in reverse order so iterate through it backwards
    for i in range(len(path) - 1, -1, -1):
      if i == 0:
        print(f"{path[i]}{vertex_index_to_ij(path[i])}")
      else:
        print(f"{path[i]}{vertex_index_to_ij(path[i])} -> ", end="")


def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP
  global g_MAP_SIZE_X
  global g_MAP_SIZE_Y
  global g_MAP_RESOLUTION_X
  global g_MAP_RESOLUTION_Y
  global g_NUM_X_CELLS
  global g_NUM_Y_CELLS

  g_src_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
  g_dest_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))

  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  g_MAP_SIZE_X  = 1.8  # 1.8 meters
  g_MAP_SIZE_Y = 1.2  # 1.2 meters
  g_MAP_RESOLUTION_X = 0.0015  # Each col represents 10cm
  g_MAP_RESOLUTION_Y = 0.0015  # Each row represents 10cm
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
  g_NUM_X_CELLS = 1200  # Number of columns in the grid map
  g_NUM_Y_CELLS = 800
  g_WORLD_MAP = [0] * g_NUM_Y_CELLS * g_NUM_X_CELLS

  # Convert from meters to ij coordinates using given function
  source = xy_coordinates_to_ij_coordinates(g_src_coordinates[0], g_src_coordinates[1])
  dest = xy_coordinates_to_ij_coordinates(g_dest_coordinates[0], g_dest_coordinates[1])

  # Run Dijkstra's on our randomly generated map and our starting source node
  obstacleMap = []
  for n in pixel_grid:
      for m in n:
          if m>0:
              obstacleMap.append(1)
          else:
              obstacleMap.append(0)
  g_WORLD_MAP = obstacleMap
  prevArray = run_dijkstra(ij_to_vertex_index(source[0], source[1]), obstacleMap)

  # Reconstruct the path from our random source to our random destination
  # Path takes in prevArray from dijkstra's, takes in vertex indices, returns list of vertex indices
  path = reconstruct_path(prevArray, ij_to_vertex_index(source[0], source[1]), ij_to_vertex_index(dest[0], dest[1]))

  # Print Path Information
  print(f"Source: {ij_to_vertex_index(source[0], source[1])}")
  print(f"Goal: {ij_to_vertex_index(dest[0], dest[1])}")

  if len(path) == 0 or path is None:
    print('No Path Found!')
  else:
    # Path is returned in reverse order so iterate through it backwards
    for i in range(len(path) - 1, -1, -1):
      if i == 0:
        print(f"{path[i]}{vertex_index_to_ij(path[i])}")
      else:
        print(f"{path[i]}{vertex_index_to_ij(path[i])} -> ", end="")

  # Convert path coordinates to image coordinates
  imageCoordinatesOfPath = []
  for step in path:
    print(obstacleMap[step])
    # Convert each path vertex index to i,j coords and then world coordinates to correspond with our image
    i, j = vertex_index_to_ij(step)
    x, y = ij_coordinates_to_xy_coordinates(i, j)

    # Convert new world coordinates to image coordinates by figuring out where that world coordinate is on our map image
    # round((meter position of point / total meters) * total cells) = proportion of image/map for that x/y (assuming equal size)
    x2 = ceil((x / g_MAP_SIZE_X) * g_NUM_X_CELLS)
    y2 = g_NUM_Y_CELLS - ceil((y / g_MAP_SIZE_Y) * g_NUM_Y_CELLS)
    imageCoordinatesOfPath.append((x2, y2))

  # Draw our path
  img = Image.open(args.obstacles)
  draw = ImageDraw.Draw(img)
  print(f"(x, y) Image Coordinates of Source: {imageCoordinatesOfPath[-1]}")
  print(f"(x, y) Image Coordinates of Destination: {imageCoordinatesOfPath[0]}")
  start = imageCoordinatesOfPath[0]
  for n in range(len(imageCoordinatesOfPath) - 1):
      line = []
      line.append(start)
      line.append(imageCoordinatesOfPath[n])
      draw.line(line, fill=255, width=3)
      start = imageCoordinatesOfPath[n + 1]

  img.save('answerC.jpg')

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.225, 0.975], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test2.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()


  #part_1()
  part_2(args)
