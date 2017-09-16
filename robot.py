import numpy as np
import sys
sys.setrecursionlimit(11000)
class Robot(object):
	def __init__(self, maze_dim):
		'''
		Use the initialization function to set up attributes that your robot
		will use to learn and navigate the maze. Some initial attributes are
		provided based on common information, including the size of the maze
		the robot is placed in.
		'''
	
		self.location = [0, 0]
		self.heading = 'up'
		self.maze_dim = maze_dim
		
		# This list contains the four cells in the center of the maze. This is the goal
		self.goal = [
			[self.maze_dim,self.maze_dim],
			[self.maze_dim-1,self.maze_dim-1], 
			[self.maze_dim,self.maze_dim-1],
			[self.maze_dim-1,self.maze_dim]
			]

		# Store the robot's last location to help us mark dead end paths. 
		self.last_location = [0,0]

		# Initialize the variables that will represent a robot's turn
		self.clockwise = 90
		self.counter_clockwise = -90

		# Initialize the variables that will represent a robot's camera orientation
		self.left_sensor = 90
		self.right_sensor = -90
		self.front_sensor = 0

		# Initialize the variable that will tell us if it's the robot's 1st or 2nd run
		self.run = 0

		# In case the robot starts his second run on a dead end path, this variable will help
		# the robot pass through the dead end cells until it reaches a junction.
		self.first_junction=[]

		# Initialize the cells of the walls_matrix matrix to 15 (binary value 1111) which means that
		# all four sides of the cell is passable.
		self.walls_matrix = [[15 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		# Initialize the cells of the visits_matrix matrix to 0. This matrix stores the number of times
		# the robot visited each square of the maze		
		self.visits_matrix = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		# Initialize the cells of the steps matrix. This matrix records the number of steps the robot
		# spent on its way to the goal.
		self.steps_matrix = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		# Initialize the cells of the turns matrix. This matrix records that number of turns the robot
		# made on its way to the goal
		self.turns_matrix = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		# Initialize the cells of the headings matrix. This matrix records the robot's headings as it
		# makes its way to the goal
		self.headings_matrix = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		# This is the variable that will contain the list of rotations and moves that the robot will
		# follow during its second run.
		self.final_moves = []

		# Initialize the number of steps that the robot will make during each run	
		self.steps = 1

	def get_heading(self, degrees):
		headings = {90:'up', 0:'right', 270:'down', 180:'left'}
		return headings[degrees]

	def get_heading_degrees(self, heading):
		degrees = {'up':90,'right':0,'down':270,'left':180}	
		return degrees[heading]

	def is_done_exploring(self):
		if self.run == 1:
			return True
		min_visits = 1
		if self.steps > 999:
			self.run = 1
			return True
		for i in range(self.maze_dim-1, -1 , -1):
			for j in range(self.maze_dim):
				if self.visits_matrix[i][j] < min_visits:
					return False
		self.run = 1
		return True
	def is_location_visited(self,location):
		if self.visits_matrix[location[0]][location[1]] > 0:
			return True
		else:
			return False		
	def get_edge_data(self, location):
		row = location[0]
		col = location[1]
		return self.walls_matrix[row][col]

	def get_edges_dict(self, location):
		# Called by update_wall_data() to conveniently work on a specific location's edge data.
		#
		# The function looks up the location's wall information from the self.maze_walls matrix.
		# The function returns a dictionary of 4 values - one for each side of the square cell
		# on the specified location parameter. The dictionary key represents the side's heading: 
		# 90 for 'north', 0 for 'east', 270 for 'south' and 180, for 'west.'
		#
		# A dictionary value of 1 indicates that the side is open and passable by the robot. A
		# dictionary value of 0 indicates the presence of a wall and is, therefore, not passable.
		# Additionally, a dictionary value of 0 on all sides represents a square that lies in a
		# dead end path.
		#


		# Retrieve the edge information for the specified location. The matrix self.maze_walls
		# contains a 4 bit representation of each cell's edge information, stored as an interger.
		# When converted back to binary, this is how the data is interpreted:
		# The 4s bit represents data for the north edge
		# The 3s bit represents data for the east edge
		# The 2s bit represents data for the south edge
		# The 1s bit represents data for the west edge
		row = location[0]
		col = location[1]	
		edges_list = list("{0:04b}".format(
			self.walls_matrix[row][col]
			))

		# Convert the list to a dictionary for easy manipulation and return the dictionary.
		edges = {
			90:edges_list[0],
			0:edges_list[1],
			270:edges_list[2],
			180:edges_list[3] }
		return edges

	def close_edges(self):
		row = self.location[0]
		col = self.location[1]			
		self.walls_matrix[row][col] = 0	
		
	def update_wall_data(self, sensors):
		# This function is called by next_move() to set up wall information on the robot's current location.

		# Initialize row and col variables to represent the current robot location
		row = self.location[0]
		col = self.location[1]

		# A value of 0 for self.maze_walls[row][col] means that the specified cell is located on a dead end
		# path. This conditional block ensures that updates to the self.maze_walls matrix are made only if
		# the current cell has not been previously set to 0.
		if self.walls_matrix[row][col] > 0:

			# Call the get_edges_dict() function to receive a dictionary of edge information for the current
			# cell. A dictionary representation of the edges makes it easier to manipulate the values.
			edges = self.get_edges_dict([row,col])

			# Update the side that is seen by the robot's left-facing sensor
			edges[(self.get_heading_degrees(self.heading) + 90) % 360] = int(sensors[0] > 0)
			# Update the side that is seen by the robot's front-facing sensor
			edges[self.get_heading_degrees(self.heading)] = int(sensors[1] > 0)
			# Update the side that is seen by the robot's right-facing sensor
			edges[(self.get_heading_degrees(self.heading) - 90) % 360] = int(sensors[2] > 0)
		
			# Update the world_view based on new sensor readings
			self.walls_matrix[row][col] = int(
				str(edges[90]) + \
				str(edges[0]) + \
				str(edges[270]) + \
				str(edges[180])
			,2)	

	def validated_location(self, location):
		next_row = location[0]
		next_col = location[1]
		if location[0] < 0:
			next_row = 0
		elif location[0] >= self.maze_dim:
			next_row = self.maze_dim - 1
		if location[1] < 0:
			next_col = 1
		elif location[1] >= self.maze_dim:
			next_col = self.maze_dim - 1		
		return [next_row, next_col]

	def search(self,step,turns,heading,row,col):
		
		if not ([row,col] in self.goal):

			edges = self.get_edges_dict([row,col])
			possible_steps = []
			for key, value in edges.iteritems():
				next_valid_location = self.validated_location([
					row + 1 * (int(np.sin(np.deg2rad(key)))),
					col + 1 * (int(np.cos(np.deg2rad(key))))
					])
				next_row = next_valid_location[0]
				next_col = next_valid_location[1]
				
				if int(value)==1:
					if (self.steps_matrix[next_row][next_col] == 0 or self.steps_matrix[next_row][next_col] > (step + 1)) and (next_row + next_col) > 0 and not (self.walls_matrix[next_row][next_col]==0):						
						self.steps_matrix[next_row][next_col] = step + 1
						self.turns_matrix[next_row][next_col] = turns + 1 * (not (key==heading))
						self.headings_matrix[next_row][next_col] = (key+180) % 360
						self.search(step + 1, turns + 1 * (not (key==heading)), key, next_row, next_col)


	def get_goal_location(self):
		entry_point = [self.maze_dim/2, self.maze_dim/2]
		entry_steps = self.steps_matrix[self.maze_dim/2][self.maze_dim/2]

		if self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2] < entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2]
			entry_point = [self.maze_dim/2 - 1, self.maze_dim/2]
		if self.steps_matrix[self.maze_dim/2][self.maze_dim/2 - 1] < entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2][self.maze_dim/2 - 1]
			entry_point = [self.maze_dim/2, self.maze_dim/2 - 1]
		if self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2 - 1] < entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2 - 1]
			entry_point = [self.maze_dim/2 - 1, self.maze_dim/2 - 1]
		return entry_point
					
					

	def find_shortest_path(self):
		shorter_path = []
		final_moves = []
		self.search(0,0,90,0,0)

		current_location = self.get_goal_location()

		while current_location != [0,0]:
			heading = self.headings_matrix[current_location[0]][current_location[1]]
			heading_reverse = (self.headings_matrix[current_location[0]][current_location[1]] + 180) % 360
			current_location[0] += 1 * (int(np.sin(np.deg2rad(heading))))
			current_location[1] += 1 * (int(np.cos(np.deg2rad(heading))))			
			shorter_path.append(heading_reverse)

		current_move = 0
		current_heading = -1
		old_heading = -1
		current_rotation = -1
		sign = -1 # Reverse the sign to correct rotation

		while shorter_path:
			current_heading = shorter_path.pop()
			if old_heading < 0: 
				current_rotation = sign * (current_heading  - 90)
			current_move += 1
			if current_heading != old_heading and old_heading >= 0:
				self.final_moves.append([current_rotation,current_move - 1])
				current_rotation = sign * (current_heading - old_heading)
				if current_rotation == 270:
					current_rotation = - 90
				elif current_rotation == -270:	
					current_rotation = 90
				current_move = 1
				if not shorter_path:
					self.final_moves.append([current_rotation,current_move])

				
			elif current_move == 4:

				self.final_moves.append([current_rotation,current_move - 1])

				current_rotation = sign * (current_heading - old_heading)
				current_move = 1
				if not shorter_path:
					self.final_moves.append([current_rotation,current_move])


			old_heading = current_heading	

		return


	def increment_visit_count(self,location):
		row = location[0]
		col = location[1]
		self.visits_matrix[row][col] += 1
	    
	def show_walls_matrix(self):
		'''
		The function show_world_view displays the cells of the maze from the view of the robot
		'''
		col = ""
		print "Showing walls matrix..."
		for i in range(self.maze_dim - 1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 4}".format(self.walls_matrix[i][j])
			print col
			
	def show_visits(self):
		'''
		The function  displays the cells of the maze that have been visited by the robot
		'''
		col = ""
		print "Showing visits matrix.."
		for i in range(self.maze_dim-1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 4}".format(self.visits_matrix[i][j])
			print col		

	def show_steps_matrix(self):
		'''
		The function  displays the cells of the maze that have been visited by the robot
		'''
		col = ""
		print "Showing steps matrix..."
		for i in range(self.maze_dim-1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 4}".format(self.steps_matrix[i][j])
			print col	

	def show_turns_matrix(self):
		'''
		The function  displays the cells of the maze that have been visited by the robot
		'''
		col = ""
		print "Showing turns matrix..."
		for i in range(self.maze_dim-1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 4}".format(self.turns_matrix[i][j])
			print col	

	def show_headings_matrix(self):
		'''
		The function  displays the cells of the maze that have been visited by the robot
		'''
		col = ""
		print "Showing headings matrix..."
		for i in range(self.maze_dim-1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += '{:>4}'.format(self.headings_matrix[i][j])
			print col 

	def get_possible_actions(self, sensors):
		max_movement = 1
		actions = []
		rotations = [self.counter_clockwise, 0, self.clockwise] # Align rotation values to sensors
		# Get possible actions based on sensor data
		for i in range(len(sensors)):
			if sensors[i] > 0:
				possible_rotation = rotations[i]
				if sensors[i] >= max_movement:
					possible_movement = max_movement
				else:
					possible_movement = sensors[i]
				actions.append([
					possible_rotation,
					possible_movement])	
		return actions				

	def initial_run_move(self, sensors):
		dead_end = False
		dead_end_penalty = 50
		possible_actions = []
		possible_rotation = 0
		possible_movement = 0
		possible_heading_degrees = 90
		possible_location = [0,0]
		sort_key = 0
		sortable_actions = []
		min_sort_key = 0
		better_actions = []
		next_rotation = 0
		next_movement = 0
		next_heading_degrees = 90
		next_heading = 'up'
		next_location = []		


		possible_actions = self.get_possible_actions(sensors)
		if len(possible_actions) > 1: 
			if self.first_junction == []:
				
				self.first_junction = self.last_location[:]

		if possible_actions == []:
			# Allow the robot to make a right turn when unable to move any further
			if not (self.first_junction==[]):
				dead_end = True
			possible_rotation = self.clockwise
			possible_movement = 0
			possible_actions.append([
				possible_rotation,
				possible_movement])					
		elif len(possible_actions)==1:
			# If there's only one way to go, check if last location is a, or leads to a dead end.
			# If so, then we should mark this location as part of a dead end path
			if self.get_edge_data(self.last_location) == 0:
				if not (self.first_junction==[]):
					dead_end = True

		if self.steps <= 1:
			possible_actions = []
			possible_rotation = self.clockwise
			possible_movement = 0
			possible_actions.append([
				possible_rotation,
				possible_movement])	
		
		for i in range(len(possible_actions)):
			
			possible_rotation = possible_actions[i][0]
			possible_movement = possible_actions[i][1]
			possible_heading_degrees = (self.get_heading_degrees(self.heading) + possible_rotation + (180 * possible_rotation/90)) % 360
			possible_location = self.validated_location([
				self.location[0] + possible_movement * (int(np.sin(np.deg2rad(possible_heading_degrees)))),
				self.location[1] + possible_movement * (int(np.cos(np.deg2rad(possible_heading_degrees))))
				])

			# Use the possible location's visit count as sort key
			sort_key = self.visits_matrix[possible_location[0]][possible_location[1]]

			# Avoid dead ends by adding a dead end penatly to the sort key. Can only go through if starting a run.
			if (self.get_edge_data(possible_location) == 0 \
				and possible_movement > 0 \
				and self.get_edge_data(self.last_location) > 0) \
				or possible_location == self.first_junction:
				sort_key = dead_end_penalty

			# Build up sortable_actions from possible_actions plus these extra
			# information: sort_key, possible_heading_degrees & possible_location
			
			sortable_actions.append([
				sort_key,
				possible_rotation,
				possible_movement,
				possible_heading_degrees,
				[possible_location[0],possible_location[1]]
				])	

		# Get the action(s) with the lowest key
		min_sort_key = min(sortable_actions)[0]
		better_actions = [action for action in sortable_actions if action[0]==min_sort_key]
		
		# Choose the better action to take
		if len(better_actions) == 1:
			action_choice = better_actions[0]
		else:
			action_choice = better_actions[np.random.randint(0,len(better_actions)-1)]

		# If the dead_end flag is set to True, close the edges of the current cell
		if dead_end:
			if self.get_edge_data(self.location) > 0:
				self.close_edges()
		self.increment_visit_count(self.location)

		# Set last_location to current location
		self.last_location[0] = self.location[0]
		self.last_location[1] = self.location[1]

		# Set current location to the chosen next location
		next_location = action_choice[4]	
		self.location[0] = next_location[0]
		self.location[1] = next_location[1]

		# Set next rotation, movement, and heading to 					
		next_rotation = action_choice[1]
		next_movement = action_choice[2]
		next_heading_degrees = action_choice[3]
		next_heading = self.get_heading(next_heading_degrees)
		self.heading = next_heading

		return next_rotation, next_movement

	def final_run_move(self):
		next_rotation, next_movement = self.final_moves[self.steps-1]
		return next_rotation, next_movement	

	def next_move(self, sensors):
		'''
		Use this function to determine the next move the robot should make,
		based on the input from the sensors after its previous move. Sensor
		inputs are a list of three distances from the robot's left, front, and
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

		# Update the edges of the robot's current location - first entry is 8 at [0,0]
		self.update_wall_data(sensors)

		# If robot is on it's first run
		if self.run == 0:		
			if self.is_done_exploring():						
				self.visits_matrix = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]
				self.location = [0,0]
				self.heading = 'up'
				self.steps = 1
				self.find_shortest_path()
				self.show_walls_matrix()
				self.show_steps_matrix()
				self.show_turns_matrix()
				self.show_headings_matrix()
				if self.steps < 1000:
					return ('Reset', 'Reset')				
			else:
				rotation, movement = self.initial_run_move(sensors)
		else:
			rotation, movement = self.final_run_move()

		self.steps += 1

		return rotation, movement