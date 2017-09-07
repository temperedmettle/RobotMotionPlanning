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
		
		self.clockwise = 90
		self.counter_clockwise = -90
		# Is the robot exploring?
		self.exploring = True

		# Robot's view of the world - each cell contains a two-value list containing a visit
		# counter and a number representing the edges
		self.world_edges = [[15 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]
		self.world_visited = [[0 for row in range(self.maze_dim)]  for col in range (self.maze_dim)]

		self.heading_degrees = 90 # 90 for up, 0 for left, 270 for down, and 180 for left		
		self.steps = 0

	def check_if_done_exploring(self):
		min_visits = 2
		self.exploring = False
		print "Checking if robot is done exploring"
		if self.steps > 999:
			return
		for i in range(self.maze_dim-1, -1 , -1):
			for j in range(self.maze_dim):
				if self.world_visited[i][j] < min_visits:
					self.exploring = True
					return
								
	def get_edge_data(self,location):
		row = self.location[0]
		col = self.location[1]
		edges_list = list("{0:04b}".format(
			self.world_edges[row][col]
			))
		# Converting list to dictionary for easy updating	
		edges = {90:edges_list[0],
			0:edges_list[1],
			270:edges_list[2],
			180:edges_list[3] }  # top, right, bottom, left
		return edges
		
	def set_edge_data(self, sensors):
		row = self.location[0]
		col = self.location[1]
		
		# Get previous edge information for current location
		edges = self.get_edge_data([row,col])
				 
		# Identify walls
		#if sensors[0] > 0: # robot's left-facing sensor
		edges[(self.heading_degrees + 90) % 360] = int(sensors[0] > 0)
		#if sensors[1] > 0: # robot's front-facing sensor
		edges[self.heading_degrees] = int(sensors[1] > 0)
		#if sensors[2] > 0: # robot's right-facing sensor
		edges[(self.heading_degrees - 90) % 360] = int(sensors[2] > 0)
		
		# Update the world_view based on new sensor readings
		self.world_edges[row][col] = int(
			str(edges[90]) + \
			str(edges[0]) + \
			str(edges[270]) + \
			str(edges[180])
		,2)	
		# print "location:[", row, ", ", col, "], heading:", self.heading_degrees, " Edges:", self.world_edges[row][col], "Sensors:", sensors

	def set_visited(self):
		row = self.location[0]
		col = self.location[1]
		self.world_visited[row][col] += 1
	    
	def show_world_edges(self):
		'''
		The function show_world_view displays the cells of the maze from the view of the robot
		'''
		col = ""
		print "Showing world view..."
		for i in range(self.maze_dim - 1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 3}".format(self.world_edges[i][j])
			print col
			
	def show_world_visited(self):
		'''
		The function  displays the cells of the maze that have been visited by the robot
		'''
		col = ""
		print "Showing visited cells..."
		for i in range(self.maze_dim-1, -1 , -1):
			col = ""
			for j in range(self.maze_dim):
				col += "{0: 3}".format(self.world_visited[i][j])
				#if [i,j] == self.location:
				#	col += "{0: 3}".format(99)
				#else:
				#	col += "{0: 3}".format(self.world_visited[i][j])
			print col		
			
	def next_exploration_move(self, sensors):	
		#action_choice = []	
		possible_actions = []
		better_actions_interim = []
		better_actions = []
		possible_location = [0,0]
		max_movement = 1

		if sensors[0] > 0: # robot's left-facing sensor
			possible_rotation = self.counter_clockwise
			if sensors[0] >= max_movement:
				possible_movement = max_movement
			else:
				possible_movement = sensors[0]
			possible_actions.append([
				possible_rotation,
				possible_movement])				
		if sensors[1] > 0: # robot's front-facing sensor
			possible_rotation = 0
			if sensors[1] >= max_movement:
				possible_movement = max_movement
			else:
				possible_movement = sensors[1]
			possible_actions.append([
				possible_rotation,
				possible_movement])				
		if sensors[2] > 0: # robot's right-facing sensor
			possible_rotation = self.clockwise
			if sensors[2] >= max_movement:
				possible_movement = max_movement
			else:
				possible_movement = sensors[2]
			possible_actions.append([
				possible_rotation,
				possible_movement])				
		if possible_actions == [] or self.steps == 0: # make a right turn
			possible_rotation = self.clockwise
			possible_movement = 0
			possible_actions.append([
				possible_rotation,
				possible_movement])			
				
		
		for i in range(len(possible_actions)):
			possible_rotation = possible_actions[i][0]
			possible_movement = possible_actions[i][1]
			possible_heading_degrees = (self.heading_degrees + possible_rotation + (180 * possible_rotation/90)) % 360
			possible_location[0] = self.location[0]
			possible_location[1] = self.location[1]
			possible_location[0] += possible_movement * (int(np.sin(np.deg2rad(possible_heading_degrees))))
			if possible_location[0] >= self.maze_dim:
				possible_location[0] = self.maze_dim - 1
			elif possible_location[0] < 0:
				possible_location[0] = 0
			possible_location[1] += possible_movement * (int(np.cos(np.deg2rad(possible_heading_degrees))))
			if possible_location[1] >= self.maze_dim:
				possible_location[1] = self.maze_dim - 1
			elif possible_location[1] < 0:
				possible_location[1] = 0
			
			better_actions_interim.append([
				self.world_visited[possible_location[0]][possible_location[1]],
				possible_rotation,
				possible_movement,
				possible_heading_degrees,
				[possible_location[0],possible_location[1]]
				])
	 	#print "BAI", better_actions_interim
		min_visits = min(better_actions_interim)[0]
		#print "MV", min_visits
		better_actions = [action for action in better_actions_interim if action[0]==min_visits]
		
		#print "BA", better_actions
		if len(better_actions) == 1:
			action_choice = better_actions[0]
		else:
			action_choice = better_actions[np.random.randint(0,len(better_actions)-1)]
		#print action_choice	
		next_rotation = action_choice[1]
		next_movement = action_choice[2]
		next_heading_degrees = action_choice[3]
		next_location = action_choice[4]
		self.set_visited()
		self.location[0] = next_location[0]
		self.location[1] = next_location[1]
		print "Next move:", action_choice , " at step", self.steps
		return next_rotation, next_movement, next_heading_degrees, next_location
		
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
	
		
		#old_heading = self.heading
		
			
	
		# Update the robot's view of the world - first entry is 8 at [0,0]
		self.set_edge_data(sensors)
		rotation, movement, self.heading_degrees, self.location = self.next_exploration_move(sensors)
		if self.exploring:
			self.check_if_done_exploring()
			if not self.exploring:
				self.show_world_edges()
				self.show_world_visited()
				
				return ('Reset', 'Reset')
		self.steps += 1
				
		#possible_heading_degrees = self.heading_degrees
		#possible_location = self.location
		#unexplored_move_directions = []
		#possible_move_directions = []
		#if sensors[0] > 0:
		#	possible_move_directions.append(counter_clockwise)
		#	possible_location[0] += movement * (int(np.sin(np.deg2rad(self.heading_degrees + counter_clockwise + (180 * counter_clockwise/90)))))
		#	possible_location[1] += movement * (int(np.cos(np.deg2rad(self.heading_degrees + counter_clockwise + (180 * counter_clockwise/90)))))
		#	if self.world_view[possible_location[0]][possible_location[1]] == 0:
		#		unexplored_move_directions.append(counter_clockwise)
		#		
		#if sensors[1] > 0:
		#	possible_move_directions.append(straight)
		#	possible_location[0] += movement * (int(np.sin(np.deg2rad(self.heading_degrees + straight + (180 * straight/90)))))
		#	possible_location[1] += movement * (int(np.cos(np.deg2rad(self.heading_degrees + straight + (180 * straight/90)))))
		#	if self.world_view[possible_location[0]][possible_location[1]] == 0:
		#		unexplored_move_directions.append(counter_clockwise)
		#
		#if sensors[2] > 0:
		#	possible_move_directions.append(clockwise)
		#	possible_location[0] += movement * (int(np.sin(np.deg2rad(self.heading_degrees + clockwise + (180 * clockwise/90)))))
		#	possible_location[1] += movement * (int(np.cos(np.deg2rad(self.heading_degrees + clockwise + (180 * clockwise/90)))))
		#	if self.world_view[possible_location[0]][possible_location[1]] == 0:
		#		unexplored_move_directions.append(clockwise)
		#if unexplored_move_directions == []:
		#	if possible_move_directions == []:
		#		rotation = np.random.choice([clockwise, counter_clockwise])
		#		movement = 0
		#	else:
		#		rotation = np.random.choice(possible_move_directions)
		#		movement = 1
		#else:
		#	rotation = np.random.choice(unexplored_move_directions)
		#	movement = 1
		#self.heading_degrees = (self.heading_degrees + rotation + (180 * rotation/90)) % 360  # added heading correction
		#self.location[0] += movement * (int(np.sin(np.deg2rad(self.heading_degrees))))
		#self.location[1] += movement * (int(np.cos(np.deg2rad(self.heading_degrees))))





		# print "loc:", self.location, ", step", self.steps, " -> old heading:", old_heading , ", turn:", {-90:'ccw', 0:'no', 90:'cw'}[rotation], ", step:", movement, ", new heading:", {90:'up',0:'right',270:'down',180:'left'}[self.heading_degrees]
		

		return rotation, movement