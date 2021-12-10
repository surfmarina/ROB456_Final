



import matplotlib.image as mpimg
import matplotlib.pyplot as plt


import numpy as np
from numpy.core.numeric import Inf

class graphDataStructure:

  
  def __init__(self, img = mpimg.imread('./src/final_project/src/aStar/BRADYCHR.pgm'), unexplored = 205, occupied = 0, unoccupied = 254):

    
    self.img = img

    self.visited     = np.zeros(np.shape(self.img))
    
    self.unexplored = unexplored  
    self.occupied   = occupied    
    self.unoccupied = unoccupied  

  
  directions = {
    "S" : np.array([ 1, 0]),
    "E" : np.array([ 0, 1]),
    "N" : np.array([-1, 0]),
    "W" : np.array([ 0,-1]),
    }
  
  def shape(self):
    return np.shape(self.img)
  
  
  def visit(self, p):
    self.visited[(tuple(p))] = 1

  
  def getVisited(self, p):
    return self.visited[(tuple(p))]

  
  def read(self, p):
    if self.img[tuple(p)] == self.unoccupied and self.getVisited(p) != 1:
      return True
    else:
      return False

  
  def show(self): 
    plt.imshow(self.img)
    plt.show()

  
  def getConnected(self, p):
    unoccupiedNeighbors = []

    for key in self.directions:
      if self.read(p + self.directions[key]):
        unoccupiedNeighbors.append(p + self.directions[key])

    return unoccupiedNeighbors

if __name__ == "__main__":

  g = graphDataStructure()

  testPoint = np.array([100, 100])

  
  assert g.getVisited(testPoint) == 0

  
  g.visit(testPoint)
  assert g.getVisited(testPoint) == 1

  d = g.getConnected([100, 100])
  print(d)
  assert len(d) == 8

  
  for i in d:
    g.visit(i)
  
  d = g.getConnected([100, 100])
  assert d == []

  print(d)

  
  print("\n===Test code for edges:===")

  
  edge_coords = [108, 55]

  viable_moves = g.getConnected(edge_coords)
  print("Viable moves: ".format(viable_moves))

  
  newArray = np.array(g.shape())



  g.show()