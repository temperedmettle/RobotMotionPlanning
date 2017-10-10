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
		self.graph = MicromouseGraph(maze_dim) # Instantiate the graph object that we will be using

		# Set self.run to 0 for robot's first run. Value is 1 for second run.
		self.run = 0	
		self.steps = 1


	def explore(self, sensors):
		# This function returns the rotation and movement values for the next
		# exploratory move.


		#if self.graph.before_first_junction==() and (int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0))  < 2:
		#	self.graph.before_first_junction = self.last_location

		# Check for and remember nodes that are in dead end paths 
		if sum(sensors) == 0 or \
		(int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0)) == 2 and self.last_location in self.graph.dead_ends:
			self.graph.dead_ends.append(self.location)

		current_node = self.location
		self.graph.update_links(current_node, self.heading, sensors) # Update links to detected nodes
		self.graph.add_visit(current_node) # Increment visit count for the current node
		result = self.graph.next_exploration_move(current_node, self.heading) # Get next rotation and movement values
		rotation, movement = result[0]
		if movement: # If location is going to change, set the last location to this location
			self.last_location = self.location[:]
		self.location = result[1] # Set location attribute to the next location
		self.heading = result[2] # Set the heading attribute to the next heading 
		return (rotation, movement)

	def solve(self):
		# This function returns the rotation and movement values for the next step
		# of the best path
		return self.graph.optimal_path[self.steps]

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
		
		if self.run == 0: # If robot is exploring the maze

			# Displays the robot's path as it explores the maze. Visited cells
			# are yellow, dead ends are red, and green is the goal.
			self.graph.display_grid(self.location, self.heading, "Steps:" + str(self.steps))

			# If the goal is reached while exploring, reset the visits counter and
			# let the robot continue exploring until it reaches the starting point
			if self.location in self.graph.goal_nodes:
				self.graph.goal_explored = True # Robot has reached the goal area
				self.graph.set_goal_entry_node(self.location) # Set the goal area's entry point
				for node in self.graph.node_visits:
					self.graph.node_visits[node] = 0

			# If the goal is reached and the robot has returned to the starting location,
			# prepare the robot to begin the second run.
			if self.graph.goal_explored and self.location == (0,0):
				self.run = 1 # Set to second run
				print "exploratory steps",self.steps		
				self.graph.optimal_path = self.graph.get_path('A*') # Generate optimal path using A*
				print "# of moves", len(self.graph.optimal_path)
				print "optimal path nodes",len(self.graph.optimal_path_nodes)
				print "optimal path headings",len(self.graph.optimal_path_headings)

				# Show the optimal path to the goal.
				self.graph.display_grid(self.location, self.heading, 
					"Exploration steps: " + str(self.steps) + "\n" \
					+ "Best path # of steps: " + str(len(self.graph.optimal_path_nodes)-1) + "\n" \
					+ "Best path # of moves: " + str(len(self.graph.optimal_path)) \
				)	
				self.steps = 0 # Reset number of steps
				return ('Reset', 'Reset') # End the first run
			else: # If robot is still exploring 
				rotation, movement = self.explore(sensors) # Get next exploration move
		else: # If robot is done exploring, get the next move from the optimal path
			rotation, movement = self.solve()
		self.steps += 1 # Increment number of steps
		return rotation, movement