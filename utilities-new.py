from Queue import PriorityQueue
import numpy as np
import os
import time

class MicromouseGraph:
	def __init__(self, size):
		self.nodes = []
		self.start_node = (0,0)
		self.goal_nodes = [
			((size / 2),(size / 2) ),
			((size / 2)-1,(size / 2)-1), 
			((size / 2),(size / 2)-1),
			((size / 2)-1,(size / 2))
			]

		# Initialize the nodes of the maze
		rows = size
		cols = size
		for x in range(cols):
			for y in range (rows):
				self.nodes.append((x,y))