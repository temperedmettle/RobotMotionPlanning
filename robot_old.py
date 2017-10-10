import numpy as np

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
		
		# This list contains the four cells in the center of the maze. This
		# is the goal area.
		self.goal = [
			[(self.maze_dim / 2),(self.maze_dim / 2) ],
			[(self.maze_dim / 2)-1,(self.maze_dim / 2)-1], 
			[(self.maze_dim / 2),(self.maze_dim / 2)-1],
			[(self.maze_dim / 2)-1,(self.maze_dim / 2)]
			]

		# Set to True is goal was reached on the first run
		self.goal_reached = False	

		# Store the robot's last location to help us mark dead end paths. 
		self.last_location = [0,0]

		# Set up values as heading adjustments to denote the orientation
		# of the robot's three sensors
		self.sensor_orientations = {'left':90, 'front':0, 'right':-90} # left, front and right

		# Set up rotation values that will be added to the robot's current
		# heading when making turns
		self.rotations = {'clockwise':90, 'none':0, 'counterclockwise':-90}

		# Set self.run to 0 for robot's first run. Value is 1 for second run.
		self.run = 0

		# In case the robot starts his second run on a dead end path, this
		# variable will help the robot pass through the dead end cells until
		# it reaches a junction.
		self.before_first_junction=[]

		# Initialize the cells of the walls_matrix matrix to 15 (binary value
		# 1111) which means that all four sides of the cell is passable.
		self.walls_matrix = [[15 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# Initialize the sensed matrix to 0 (binary 0000) which means that no
		# edges have been observed yet by the robot's sensors.			
		self.sensed_matrix = [[0 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# Set up outer edge cells which we already know have walls	
		for i in range(self.maze_dim):
			self.walls_matrix[i][0] -= 1
			self.walls_matrix[0][i] -= 2
			self.walls_matrix[i][self.maze_dim-1] -= 4
			self.walls_matrix[self.maze_dim-1][i] -= 8

			self.sensed_matrix[i][0] += 1
			self.sensed_matrix[0][i] += 2
			self.sensed_matrix[i][self.maze_dim-1] += 4
			self.sensed_matrix[self.maze_dim-1][i] += 8	

		# Initialize the cells of the visits_matrix matrix to 0. This matrix
		# stores the number of times the robot visited each square of the maze		
		self.visits_matrix = [[0 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# Initialize the cells of the steps matrix. This matrix records the
		# number of steps the robot spent on its way to the goal.
		self.steps_matrix = [[0 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# Initialize the cells of the turns matrix. This matrix records that
		# number of turns the robot made on its way to the goal
		self.turns_matrix = [[0 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# Initialize the cells of the headings matrix. This matrix records
		# the robot's headings as it makes its way to the goal
		self.headings_matrix = [[0 for y in range(self.maze_dim)]  \
			for x in range (self.maze_dim)]

		# This is the variable that will contain the list of rotations and
		# moves that the robot will follow during its second run.
		self.final_moves = []

		# Initialize the number of steps that the robot will make during
		# each run	
		self.steps = 1

	def is_goal_reached(self):
		# Check if robot has already entered the goal area. Called inside
		# is_done_exploring()
		if self.goal_reached:
			return True
		else:
			for y, x in self.goal:
				if self.visits_matrix[y][x] > 0:

					#self.visits_matrix = [[0 for y in range(self.maze_dim)]  \
					#	for x in range (self.maze_dim)]

					self.goal_reached = True
					return True
			return False
		
	def distance_to_origin(self,location):
		return location[0] + location[1]

	def distance_to_goal(self,location):
		# Called inside initial_run_move() to create the sort_key that will
		# help the robot decide which direction to take when in a junction
		distance = (self.maze_dim / 2) - 1
		for y, x in self.goal:
			possible_distance = abs(y - location[0]) + abs(x - location[1])
			if possible_distance < distance:
				distance = possible_distance		
		return possible_distance	

	def get_heading_text(self, degrees):
		# This function is called by initial_run_move() to update the global
		# variable self.heading
		headings = {90:'up', 0:'right', 270:'down', 180:'left'}
		return headings[degrees]

	def get_heading_degrees(self, heading):
		# Called by update_edge_data() and initial_run_move() functions to
		# get the degrees value of the text heading
		degrees = {'up':90,'right':0,'down':270,'left':180}	
		return degrees[heading]

	def is_done_exploring(self):
		# This function is called by next_move() to check if the robot has
		# finished mapping the maze or has reached the maximum number of
		# steps allowed. Additionally, the function sets the global variable
		# self.run to 1 to denote the start of the second run.
		
		if self.run == 1: # Robot is done exploring if it is on it's 2nd run 
			return True
		if self.run == 0 and self.steps > 999: # Exploration ends at 1000 steps
			self.run = 1
			return True
		if self.is_goal_reached() and self.distance_to_origin(self.location)==0: # End exploration when goal is reached
			self.run = 1
			for y in range(self.maze_dim-1, -1 , -1): 
				for x in range(self.maze_dim):
					if self.sensed_matrix[y][x] < 15:
						sensed = self.get_edges_dict([y,x],'sensed') 
						walls = self.get_edges_dict([y,x],'wall')
						for key in sensed:
							if sensed[key] == '0':
								walls[key] = 0
						self.walls_matrix[y][x] = int(
							str(walls[90]) + \
							str(walls[0]) + \
							str(walls[270]) + \
							str(walls[180])
						,2)
			return True
		return False

	def get_edge_data(self, location):
		# This function is called inside initial_run_move() to identify
		# dead end paths
		y = location[0]
		x = location[1]
		return self.walls_matrix[y][x]

	def get_edges_dict(self, location, info):
		# Called by update_sensed_edges() and search() to conveniently work 
		# on a specific location's edge data. The type of edge data returned 
		# is either wall data or sensed data as indicated in the 'info'
		# parameter.
		#
		# The function looks up the location's wall (or sensed) information.
		# The function returns a dictionary of 4 values - one for each side of
		# of the square cell on the specified location parameter.
		# The dictionary key represents the side's heading: 
		# 90 for 'north', 0 for 'east', 270 for 'south' and 180, for 'west.'
		#
		# Wall data:
		# A dictionary value of 1 indicates that the side is open and passable
		# by the robot. A dictionary value of 0 indicates the presence of a wall
		# and is, therefore, not passable. Additionally, a dictionary value of
		# 0 on all sides represents a square that lies in a dead end path.
		#
		# Sensed data:
		# A dictionary value of 1 indicates that a cell edge has been previously
		# observed by the robot.
		#
		# The matrices self.walls_matrix and self.sensed_matrix contains a
		# 4-bit representation of each cell's edge information in integer
		# format. When converted back to binary, data is interpreted as follows:
		# 
		# The 4s bit represents data for the north edge
		# The 3s bit represents data for the east edge
		# The 2s bit represents data for the south edge
		# The 1s bit represents data for the west edge
		# 
		# For example, a value of 15 (binary 1111) in a cell of the wall matrix
		# means that there are not walls on all four edges.

		y = location[0]
		x = location[1]	
		if info == 'wall': 
			edges_list = list("{0:04b}".format(self.walls_matrix[y][x]))
		else:
			edges_list = list("{0:04b}".format(self.sensed_matrix[y][x]))
				
		# Convert the list to a dictionary for easy manipulation and return
		# the dictionary.
		edges = {
			90:edges_list[0],
			0:edges_list[1],
			270:edges_list[2],
			180:edges_list[3] }
		return edges

	def close_edges(self, location):
		# This function is called inside initial_run_move() and update_sensed_edges()
		# to 'tag' a cell as part of a dead end path.
		y = location[0]
		x = location[1]			
		self.walls_matrix[y][x] = 0	
		self.sensed_matrix[y][x] = 15	

	def is_out_of_bounds(self, location):
		# This function is called in update_sensed_edges() to check if a cell is
		# is beyond the maze's dimensions
		if location[0] < 0 or location[0] >= self.maze_dim \
			or location[1] < 0 or location[1] >= self.maze_dim:
			return True
		return False

	def update_sensed_edges(self, location, heading, current_step, last_step):
		# This function is called in update_edge_data() to update the sensed matrix
		# based on the robot's sensor readings at the current location. This
		# recursive function returns a list containing a walls_matrix and 
		# sensed_matrix cell value, or 'None' if coordinates are outside the maze.

		this_y = location[0]
		this_x = location[1]	

		# If location parameter is beyond the maze, return 'None' and exit
		# immediately
		if self.is_out_of_bounds([this_y,this_x]):
			return None

		# Compute the next cell observed by the robot's sensor
		next_y = this_y + 1 * (int(np.sin(np.deg2rad((heading) % 360))))
		next_x = this_x + 1 * (int(np.cos(np.deg2rad((heading) % 360))))
		
		# Get wall and sensed data
		wall_data = self.get_edges_dict([this_y,this_x], 'wall')
		sensed_data = self.get_edges_dict([this_y,this_x], 'sensed')

		# If cell is a dead end, put up a 'virtual wall' to prevent future passage
		if self.walls_matrix[this_y][this_x] in [8, 4, 2, 1, 0] \
			and (this_y + this_x) > 0:
			self.close_edges([this_y, this_x])
		else:
			# Mark as passable the edges of cells that the robot can 'see through' 
			if current_step > 0 and (this_y + this_x) > 0:
				wall_data[(heading + 180) % 360] = 1
				sensed_data[(heading + 180) % 360] = 1

			if current_step < last_step - 1:
				wall_data[heading] = 1
				sensed_data[heading] = 1

			# Update edge and sensed data on the cell where the robot found a wall 
			if current_step == last_step - 1:
				wall_data[heading] = 0
				sensed_data[heading] = 1

			if current_step == last_step:
				wall_data[(heading + 180) % 360] = 0
				sensed_data[(heading + 180) % 360] = 1

			# Push the data into walls and sensed matrices	
			self.walls_matrix[this_y][this_x] = int(
				str(wall_data[90]) + \
				str(wall_data[0]) + \
				str(wall_data[270]) + \
				str(wall_data[180])
			,2)

			self.sensed_matrix[this_y][this_x] = int(
				str(sensed_data[90]) + \
				str(sensed_data[0]) + \
				str(sensed_data[270]) + \
				str(sensed_data[180])
			,2)	

		if current_step == last_step:
			# This is the cell just after the one where the robot sensed a wall
			return [self.walls_matrix[this_y][this_x], self.sensed_matrix[this_y][this_x]]
		else:
			# Call this function using the next location and incremented step as parameters
			next_cell = self.update_sensed_edges(
				[next_y,next_x],
				heading,
				current_step + 1,
				last_step)
			if not (next_cell == None):
				if (next_cell[0] == 0) \
					and (current_step < last_step - 1) \
					and (sensed_data[(heading + 90) % 360] + sensed_data[(heading - 90) % 360] == 2) \
					and (wall_data[(heading + 90) % 360] + wall_data[(heading - 90) % 360] == 0):
					# Close this cell if it leads to a dead end
					self.close_edges([this_y, this_x])
			# Return the cell values on both walls and sensed matrices as a list	
			return [self.walls_matrix[this_y][this_x], self.sensed_matrix[this_y][this_x]]	

	def update_edge_data(self, sensors):
		# This function is called by next_move() to set up wall information on
		# the robot's current location.

		# Initialize x and y variables to represent the current robot
		# location
		y = self.location[0]
		x = self.location[1]
		
		# Go through the cells observed by the robot's sensors and update edge
		# data for these cells 
		
		orientations = [
			self.sensor_orientations['left'],
			self.sensor_orientations['front'],
			self.sensor_orientations['right']]

		for i in range(len(orientations)):
			self.update_sensed_edges( 
				self.location, 
				(self.get_heading_degrees(self.heading) + orientations[i]) % 360, 
				0, 
				sensors[i]+1)

	def validated_location(self, location):
		# Called from search() and initial_run_move(), this function makes
		# sure that the x and y position fall within the maze's boundaries
		# For example, a negative x or y value is changed to 0, or a y of
		# 16 on a 16x16 maze will be changed to 15 since valid values will
		# be from 0 to 15.  
		next_y = location[0]
		next_x = location[1]
		if location[0] < 0:
			next_y = 0
		elif location[0] >= self.maze_dim:
			next_y = self.maze_dim - 1
		if location[1] < 0:
			next_x = 1
		elif location[1] >= self.maze_dim:
			next_x = self.maze_dim - 1		
		return [next_y, next_x]

	def search(self,step,turns,heading,y,x):
		# This is a recursive function that will help the robot find the
		# shortest path to the goal. Initially called inside the
		# find_shortest_path() function.
		if not ([y,x] in self.goal):

			wall_data = self.get_edges_dict([y,x], 'wall')
			possible_steps = []

			# Loop through each edge of the square cell. The variable key
			# represents the heading, while value refers to the binary
			# that indicates whether an edge contains a wall (0) or not (1).
			for key, value in wall_data.iteritems():
				next_valid_location = self.validated_location([
					y + 1 * (int(np.sin(np.deg2rad(key)))),
					x + 1 * (int(np.cos(np.deg2rad(key))))
					])
				next_y = next_valid_location[0]
				next_x = next_valid_location[1]
				
				# Check if the robot can pass through this side of the square 
				# (no wall to stop it)
				if int(value)==1:

					# If next square on this path has not been used by a
					# shorter path, continue with this path. 
					if (self.steps_matrix[next_y][next_x] == 0\
						or self.steps_matrix[next_y][next_x] > (step + 1) \
						or (self.steps_matrix[next_y][next_x] == (step + 1) \
						and self.turns_matrix[next_y][next_x] > turns + 1)  ) \
						and (next_y + next_x) > 0 \
						and not (self.walls_matrix[next_y][next_x]==0):

							# Update the next square on the steps, turns and
							# headings matrices.
							self.steps_matrix[next_y][next_x] = step + 1
							self.turns_matrix[next_y][next_x] = turns + 1 \
								* (not (key==heading))
							self.headings_matrix[next_y][next_x] = (key+180) % 360

							# Perform the previous steps on the next square
							self.search(step + 1, turns + 1 \
								* (not (key==heading)), key, next_y, next_x)




	def get_goal_location(self):
		# Called inside the find_shortest_path() function, get_goal_location
		# returns he goal cell that the robot will enter first.
		entry_point = [self.maze_dim/2, self.maze_dim/2]
		entry_steps = self.steps_matrix[self.maze_dim/2][self.maze_dim/2]

		if self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2] > entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2]
			entry_point = [self.maze_dim/2 - 1, self.maze_dim/2]
		if self.steps_matrix[self.maze_dim/2][self.maze_dim/2 - 1] > entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2][self.maze_dim/2 - 1]
			entry_point = [self.maze_dim/2, self.maze_dim/2 - 1]
		if self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2 - 1] > entry_steps:
			entry_steps = self.steps_matrix[self.maze_dim/2 - 1][self.maze_dim/2 - 1]
			entry_point = [self.maze_dim/2 - 1, self.maze_dim/2 - 1]
		return entry_point	

	def find_shortest_path(self):
		# Called inside next_move(), this function calls search() to do a
		# recursive path-finding routine, lists all the steps from goal to
		# start, then groups small steps into a bigger one (i.e. 3 single
		# steps in a straight line can be combined into one move)
		shortest_path = []
		final_moves = []

		# The robot may only move up to three consecutive steps in a straight
		# line.
		max_steps = 3 

		current_move = 0
		current_heading = -1
		old_heading = -1
		current_rotation = -1
		sign = -1 # Reverse the sign to correct rotation

		# Start the recursive search for the shortest path
		self.search(0,0,90,0,0)

		# Use the headings matrix to trace our steps starting at the goal
		# all the wayback to the starting location. Record the reversed
		# heading to the shortest_path list.
		current_location = self.get_goal_location()
		
		while current_location != [0,0]:

			heading = self.headings_matrix[current_location[0]][current_location[1]]
			heading_reverse = \
				(self.headings_matrix[current_location[0]][current_location[1]] \
				+ 180) % 360
			current_location[0] += 1 * (int(np.sin(np.deg2rad(heading))))
			current_location[1] += 1 * (int(np.cos(np.deg2rad(heading))))
			current_location = self.validated_location(current_location)

			shortest_path.append(heading_reverse)

		# Go through the shortest_path list from last to first element
		# (starting point to goal), combining steps into moves and calculating
		# rotation. Store the rotation and movement to the final_moves list.
		# The self.final_moves list will be used by the final_run_move()
		# function to provide rotation and movement values to the next_move()
		# function.

		while shortest_path:
			current_heading = shortest_path.pop()
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
				if not shortest_path:
					self.final_moves.append([current_rotation,current_move])				
			elif current_move == max_steps + 1:
				self.final_moves.append([current_rotation,current_move - 1])
				current_rotation = sign * (current_heading - old_heading)
				current_move = 1
				if not shortest_path:
					self.final_moves.append([current_rotation,current_move])
			elif current_heading == old_heading:
				if not shortest_path:
					self.final_moves.append([current_rotation,current_move])		
			old_heading = current_heading	

		return

	def increment_visit_count(self,location):
		# This is used by the initial_run_move() function to increment the
		# visits matrix
		y = location[0]
		x = location[1]
		self.visits_matrix[y][x] += 1
	    
	def show_walls_matrix(self):
		'''
		The function show_world_view displays the cells of the maze from
		# the view of the robot
		'''
		
		print "Showing walls matrix..."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.walls_matrix[i][j])
			print row
			
	def show_visits_matrix(self):
		'''
		The function  displays the cells of the maze that have been
		visited by the robot
		'''
		
		print "Showing visits matrix.."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.visits_matrix[y][x])
			print row	

	def show_steps_matrix(self):
		'''
		The function  displays the cells of the maze that have been
		visited by the robot
		'''
		
		print "Showing steps matrix..."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.steps_matrix[y][x])
			print row

	def show_turns_matrix(self):
		'''
		The function  displays the cells of the maze that have been
		visited by the robot
		'''
		
		print "Showing turns matrix..."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.turns_matrix[y][x])
			print row

	def show_headings_matrix(self):
		'''
		The function  displays the cells of the maze that have been
		visited by the robot
		'''
		
		print "Showing headings matrix..."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.headings_matrix[y][x])
			print row

	def show_sensed_matrix(self):
		'''
		The function displays the cells of the maze whose walls have
		been sensed by the robot
		'''
		
		print "Showing sensed matrix..."
		for y in range(self.maze_dim - 1, -1 , -1):
			row = ""
			for x in range(self.maze_dim):
				row += "{0: 4}".format(self.sensed_matrix[y][x])
			print row		

	def get_possible_actions(self, sensors):
		# Called inside initial_run_move(), this function returns a 
		# list of possible rotation and movement values based on the
		# robot's available sensors.
		max_movement = 1
		actions = []

		# Align rotation values to sensors
		rotations = [self.rotations['counterclockwise'],
					 self.rotations['none'],
					 self.rotations['clockwise']] 

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
		# This is called by the next_move() function during the robot's
		# first run, when it attempts to map the maze.
		dead_end = False
		dead_end_penalty = 160000
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
			if self.before_first_junction == []:	
				self.before_first_junction = self.last_location[:]

		if possible_actions == []:
			# Allow the robot to make a right turn when unable to move
			# any further. Add this move to possible_actions
			if not (self.before_first_junction==[]):
				dead_end = True
			possible_rotation = self.rotations['clockwise']
			possible_movement = 0
			possible_actions.append([
				possible_rotation,
				possible_movement])					
		elif len(possible_actions)==1:
			# If there's only one way to go, check if last location is
			# a, or leads to a dead end. If so, then we should mark this
			# location as part of a dead end path
			if self.get_edge_data(self.last_location) == 0:
				if not (self.before_first_junction==[]):
					dead_end = True
		
		for i in range(len(possible_actions)):
			
			possible_rotation = possible_actions[i][0]
			possible_movement = possible_actions[i][1]
			possible_heading_degrees = \
				(self.get_heading_degrees(self.heading) 
				+ possible_rotation 
				+ (180 * possible_rotation/90)) % 360
			possible_location = self.validated_location([
				self.location[0] + possible_movement * 
				(int(np.sin(np.deg2rad(possible_heading_degrees)))),
				self.location[1] + possible_movement * 
				(int(np.cos(np.deg2rad(possible_heading_degrees))))
				])

			# Use the possible location's visit count as primary sort key with
			# distance to goal as the secondary sort key
			if not self.is_goal_reached():
				sort_key = self.visits_matrix[possible_location[0]][possible_location[1]] * 10000 \
					+ self.distance_to_goal(possible_location) * 100
			else:		
				sort_key = self.visits_matrix[possible_location[0]][possible_location[1]] * 10000 \
					+ self.sensed_matrix[possible_location[0]][possible_location[1]] * 100 \
					+ self.distance_to_origin(possible_location)

			# Avoid dead ends by adding a dead end penatly to the sort key.
			# Can only go through if starting a run.
			if (self.get_edge_data(possible_location) == 0 \
				and possible_movement > 0 \
				and self.get_edge_data(self.last_location) > 0) \
				or (possible_location == self.before_first_junction \
				and not self.is_goal_reached()):
				sort_key = dead_end_penalty

			# Build up sortable_actions from possible_actions plus these
			# extra information: sort_key, possible_heading_degrees &
			# possible_location
			
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

		# If the dead_end flag is set to True, close the edges of the
		# current cell
		if dead_end:
			if self.get_edge_data(self.location) > 0:
				self.close_edges(self.location)
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
		next_heading = self.get_heading_text(next_heading_degrees)
		self.heading = next_heading

		return next_rotation, next_movement

	def final_run_move(self):
		# This function is called inside next_move() during the robot's
		# second run when it attempts to reach its goal with the least
		# amount of moves.
		
		next_rotation, next_movement = self.final_moves[self.steps-1]
		return next_rotation, next_movement	

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
			self.show_visits_matrix()
			self.update_edge_data(sensors)
			if self.is_done_exploring():
				print "First run completed in", self.steps, "steps"
				self.location = [0,0]
				self.heading = 'up'
				self.steps = 1
				self.show_sensed_matrix()
				# self.show_walls_matrix()

				self.visits_matrix = [[0 for y in range(self.maze_dim)]  \
					for x in range (self.maze_dim)]				
				self.find_shortest_path()
				self.show_steps_matrix()

				print "Computed optimal path has", len(self.final_moves), "moves." 
				if self.steps < 1000:

					return ('Reset', 'Reset')				
			else:
				rotation, movement = self.initial_run_move(sensors)
		else:
			
			rotation, movement = self.final_run_move()
			
		self.steps += 1

		return rotation, movement