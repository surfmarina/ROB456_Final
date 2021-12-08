from os import path
import numpy as np
from numpy.core.numeric import Inf, array_equal

import floodFill as ff
import graphDataStructure as gsd

import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import cProfile as cP

class aStar:
  def __init__(self, start, goal, h = lambda p1, p2: np.linalg.norm(p2-p1[0])+p1[1]) -> None:
    self.start  = start
    self.goal   = goal
    # Everything happens inside floodfill, the two lambda functions are passed in and modify the behavior turning it into floodfill
    self.ff = ff.floodFill(startLoc = start, endLoc = goal, e = lambda p0, p1, p2: np.array_equal(p1, p2), w = h)

  # This is a bad implementation. It works, but it's _really_ bad
  def path(self, out):
    # Container for building the path
    path = []
    # pc is a point used to keep track of where we are, start at the end and move towards the begining
    pc = self.goal

    # Keep going until current point is the starting point, 
    # this doesnt actually work, since the starting point is an invalid move
    # I have fixed this with the (if -> break) down below
    while np.array_equal(pc, self.start) == False:
      # make a new point to test the different neighbors, this is so we don't loose track of where we are
      pn = pc

      # due to a bug in getViableNeighbors, manually loop from the direction variable
      # This behavior is subject to change (but not tonight)
      for key in self.ff.gds.directions:

        # point is the sum of current position and direction
        pKey = pn + self.ff.gds.directions[key]
        # if neighbor is a lower than our current selection, select it
        # since this hits all neigbors, it is assured to select the minimal neighbor
        if out[tuple(pKey)] < out[tuple(pn)] and out[tuple(pKey)] != 0:
          pn = pn + self.ff.gds.directions[key]

      # Actual exit condition, this is bad practice, but I was in a hurry
      # If there is no better move than staying still, consideder as close as possible and quit
      if np.array_equal(pn, pc):
        break
      
      # Syntactically identical to path.push pc, I just like how this look instead (spread/splat opperator)
      # *path is replaced with every item in path, so it essentially 
      # sets path to a new array containing everything in path plus the new element
      path = [*path, pc]
      # Update current point to the selected point to continue itterating
      pc = pn
  
    return path

  def run(self):
    return self.ff.run()

if __name__ == "__main__":
  a = aStar([100, 100], [250, 325])
  # a = aStar([100, 325], [250, 75])

  aOut  = a.run()                         # Run A*
  aPath = a.path(aOut)                    # Refine A* to best path
  ffOut = ff.floodFill([100, 100]).run()  # Get floodflow to use as a pretty background

  # debugging
  # print(aPath)

  # Set all points on the floodfill image that are also in the A* path to Inf (white)
  for i in aPath:
    ffOut[tuple(i)] = Inf

  # Show floodflow output with superimposed A* best path
  plt.imshow(ffOut)

  # cP.run('a.path(a.run())')

  # Invoke PLT renderer
  plt.show()