import numpy as np
from utilities import MicromouseGraph


class Robot(object):
	def __init__(self, maze_dim):
		'''
		Use the initialization function to set up attributes that your robot
		will use to learn and navigate the maze. Some initial attributes are
		provided based on common information, including the size of the maze
		the robot is placed in.
		'''
	
		self.location = (0, 0)
		self.heading = 'up'
		self.maze_dim = maze_dim



		self.last_location = (0,0)
		self.graph = MicromouseGraph(maze_dim)

		# Set self.run to 0 for robot's first run. Value is 1 for second run.
		self.run = 0	
		self.steps = 1


	def explore(self, sensors):

		if sum(sensors) == 0 or \
		(int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0)) == 2 and self.last_location in self.graph.dead_ends:
			self.graph.dead_ends.append(self.location)

		self.graph.debug_print(("Step", self.steps))
		current_node = self.location
		self.graph.update_links(current_node, self.heading, sensors)
		self.graph.add_visit(current_node)
		result = self.graph.next_exploration_move(current_node, self.heading)
		rotation, movement = result[0]
		if movement:
			self.last_location = self.location[:]	
		self.location = result[1]
		self.heading = result[2]
		self.graph.debug_print(("rot:",rotation, ", move:",movement, ", hdg:",self.heading, ", loc:", self.location))
			

		return (rotation, movement)


	def get_best_move(self):
		
		return (0,0)	

	def next_move(self, sensors):
		'''
		Use this function to determine the next move the robot should make,
		based on the input from the sensors after its previous move. Sensor
		inputs are a list of three distances from the robot's left, front,and
		right-facing sensors, in that order.

		Outputs should be a tuple of two values. The first value indicates
		robot rotation (if any), as a number: 0 for no rotation, +90 for a
		90-degree rotation clockwise, and -90 for a 90-degree rotation
		counterclockwise. Other values will result in no rotation. The second
		value indicates robot movement, and the robot will attempt to move the
		number of indicated squares: a positive number indicates forwards
		movement, while a negative number indicates backwards movement. The
		robot may move a maximum of three units per turn. Any excess movement
		is ignored.

		If the robot wants to end a run (e.g. during the first training run in
		the maze) then returing the tuple ('Reset', 'Reset') will indicate to
		the tester to end the run and return the robot to the start.
		'''
		
		rotation = 0
		movement = 0	

		# Update the edges of the robot's current location - first entry is 8
		# at [0,0]
		
		# If robot is on it's first run
		if self.run == 0:
			self.graph.display_grid(self.location, self.heading, "Steps:" + str(self.steps))
			if self.location in self.graph.goal_nodes:
				self.graph.goal_explored = True
				self.graph.set_goal_entry_node(self.location)
				self.graph.node_visits = {}
				self.graph.debug_print(("GOAL REACHED -> GOING BACK NOW"))

			if self.graph.goal_explored and self.location == (0,0):
				self.graph.debug_print(("Links",len(self.graph.links)))
				self.steps = 0
				self.run = 1
				self.graph.node_visits
				self.graph.get_path('A*')
				return ('Reset', 'Reset')
			else:	
				rotation, movement = self.explore(sensors)
		else:
			rotation, movement = self.get_best_move()
		self.steps += 1

		return rotation, movement