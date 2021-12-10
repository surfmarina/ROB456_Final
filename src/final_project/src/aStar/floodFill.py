
import numpy as np
from numpy.core.numeric import Inf

import priority_queue as pq
import graphDataStructure as gsd

import matplotlib.image as mpimg
import matplotlib.pyplot as plt

# floodFill takes a graphDataStructure, a priority queue and a partial weighting algorithm
# to generate an array of equal size to the origional image containing the flood of the space
# weighted by the weighting algorithm and normalized by the highest value to [0, 512] (the range of the imshow colorspace)
class floodFill:

  def __init__( self, startLoc, endLoc = [Inf, Inf],
                img = mpimg.imread('./src/final_project/src/aStar/BRADYCHR.pgm'),
                e = lambda p0, p1, p2: p0.is_empty(), 
                w = lambda p1, p2: p1[1]+1) -> None:
    # Priority Queue for flood, initially empty
    self.pq   = pq.PriorityQueue()

    # Graph data structure
    self.gds  = gsd.graphDataStructure(img=img)

    # Starting Point
    loc = np.array(startLoc)
    self.endLoc = np.array(endLoc)
    # Mark Starting Point as Visited
    self.gds.visit(loc)

    # Push Starting Point
    self.pq.push((loc, 0))

    self.e = e  # lambda end condition test
    self.w = w  # lambda weighing formula

  def run(self):
    # Output of flood fill, to be returned
    out  = np.zeros(self.gds.shape())
    # While there are points in the queue
    while 1:
      # Get highest priority point
      curP = self.pq.pop()
      # Check each of it's neighbors
      for i in self.gds.getConnected(curP[0]):
        self.gds.visit(i)                             # Mark valid point as visited to prevent revisits
        out[tuple(i)] = self.w(curP, self.endLoc)     # Update output array
        self.pq.push((i, self.w(curP, self.endLoc)))  # Add point to pq for later

      # Check the exit condition function, if true, break the while loop
      if self.e(self.pq, curP[0], self.endLoc):
        break

    # Return the output map
    return out

if __name__ == "__main__":

  # Initialize floodd fill class and run to generate output
  ffOut = floodFill([250,325]).run()

  # Show
  plt.imshow(ffOut)

  # Invoke PLT renderer
  plt.show()

  pass