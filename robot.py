import numpy as np 
from Queue import PriorityQueue
import os
import time

class bcolors:
	# colors used is displaying paths
    # BLUE = '\033[94m'
    BLUE = '\033[93m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class MicromouseGraph(object):
	def __init__(self, size):
		self.nodes = []
		self.links = []
		self.goal_area = [
			((size / 2),(size / 2) ),
			((size / 2)-1,(size / 2)-1), 
			((size / 2),(size / 2)-1),
			((size / 2)-1,(size / 2))
			]
		self.start_node = (0,0)
		self.goal_node = ()
		self.dead_ends = []
		self.node_visits = {}
		self.optimal_path_nodes = []
		self.optimal_path_headings = []
		self.before_first_junction = []
		self.junctions = []
		self.steps = 0
		self.sub_goals = [(0,size-1),(size-1,size-1),(size-1,0),(size / 2 - 0.5, size / 2 - 0.5)]
		self.sub_goal = self.sub_goals[np.random.randint(len(self.sub_goals))]
		# self.sub_goal = (size-1,size-1)

		# Initialize the nodes of the maze
		rows = size
		cols = size
		for x in range(cols):
			for y in range (rows):
				self.nodes.append((x,y))
	def add_link (self, node_1, node_2):
		if not (node_1,node_2) in self.links:
			self.links.append((node_1,node_2))
		if not (node_2,node_1) in self.links:
			self.links.append((node_2,node_1))
	def neighbors (self, node):
		neighbors = []
		for (node_1, node_2) in self.links:
			if node == node_1:
				neighbors.append(node_2)
		return neighbors
	def best_path (self, node_1, node_2):
		# This function is an implementation the A* search algorithm
		frontier = PriorityQueue()
		frontier.put((0, node_1))
		from_node = {}
		from_node[node_1] = None
		cost = {}
		cost[node_1] = 0
		path = []
		current_node = ()
		while not frontier.empty():
			current_node = (frontier.get())[1]
			if not (current_node in self.goal_area):
				neighbors = self.neighbors(current_node)
				for next_node in neighbors:
					# Add a turn penalty to prefer paths with fewer turns
					last_node = current_node
					if from_node[current_node]:
						last_node = from_node[current_node] 
					turn_penalty = 1
					if (last_node[0] == current_node[0] and current_node[0] == next_node[0]) or (last_node[1] == current_node[1] and current_node[1] == next_node[1]):
						turn_penalty = 0	
					# Include turn penalty to cost	
					new_cost = cost[current_node] + turn_penalty + 1
					heuristic = abs(next_node[0] - self.goal_node[0]) + abs(next_node[1] - self.goal_node[1])					
					if next_node not in cost or new_cost < cost[next_node]:
						cost[next_node] = new_cost
						priority = new_cost + heuristic
						frontier.put((priority, next_node))
						from_node[next_node] = current_node
			else:
				break
		# Build the path list		
		path = [current_node]
		while current_node != self.start_node:			
			current_node = from_node[current_node]
			path.append(current_node)
		path.reverse()
		
		self.optimal_path_nodes = path
		return path

	def display_grid(self, location, heading, size, notes):
		# This function will display the path taken by the robot, as well as the optimal path
		# that the robot has found. The following color coding is used:
		#
		# 		yellow	= 	visited
		# 		red 	= 	dead end
		#		green 	= 	goal node or optimal path node
		#		black 	= 	the robot
		#
		# Additionally, the robot and optimal paths will include heading information 
		visited_symbol = " * "
		time.sleep(0.05)
		ARROWS = {'up':' ^ ','right':' > ','down':' v ','left':' < '}
		os.system('cls' if os.name == 'nt' else 'clear')
		for y in range(size - 1, -1 , -1):
			col = "  "
			for x in range(size):	
				if (x,y) == location:
					col += bcolors.BOLD + ARROWS[heading] + bcolors.ENDC
				else:
					if (x,y) in self.dead_ends:
						col += bcolors.RED + visited_symbol + bcolors.ENDC					
					elif (x,y) in self.node_visits:
						if (x,y) in (self.goal_area + self.optimal_path_nodes):
							char = visited_symbol
							if (x,y) in self.optimal_path_nodes:
								idx = self.optimal_path_nodes.index((x,y))
								if idx == len(self.optimal_path_headings):
									idx -= 1								
								char = ARROWS[self.optimal_path_headings[idx]]		
							col += bcolors.GREEN + char + bcolors.ENDC
						elif (x,y) in self.junctions:
							col += bcolors.BLUE + visited_symbol + bcolors.ENDC		
						else:
							col += bcolors.YELLOW + visited_symbol + bcolors.ENDC
					else:
						char = visited_symbol
						if (x,y) in self.optimal_path_nodes:
							idx = self.optimal_path_nodes.index((x,y))
							if idx == len(self.optimal_path_headings):
								idx -= 1
							char = ARROWS[self.optimal_path_headings[idx]]								
							col += bcolors.GREEN + char + bcolors.ENDC
						else:	
							col += "   "		
			print col
		print notes


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
		self.exploring = True # Values are: 0 for exploration, 1 for maze-solving
		self.graph = MicromouseGraph(self.maze_dim)
		self.MOVEMENTS = (
			{'up':(0,1),'down':(0,-1),'left':(-1,0),'right':(1,0)},
			{(0,1):'up',(0,-1):'down',(-1,0):'left',(1,0):'right'})
		self.HEADINGS = (
			{90:'up', 270:'down', 180:'left', 0:'right', -90:'down', 360:'right'},
			{'up':90, 'down':270, 'left':180, 'right':0} )		
		self.STEERING_DIRECTIONS = (-1,0,1) # counterclockwise, no turn, clockwise
		self.ROTATION_ANGLE = 90
		self.fast_run_moves = []
		self.fast_run_step = 0
		self.last_location = self.graph.start_node

	def next_location_and_heading(self, location, heading, rotation, movement):
		new_heading =  self.HEADINGS[0][self.HEADINGS[1][heading] - rotation]
		new_location = (
			location[0] + self.MOVEMENTS[0][new_heading][0] * movement,
			location[1] + self.MOVEMENTS[0][new_heading][1] * movement)  		
		return (new_location, new_heading)

	def moves_to_goal(self, path):
		# This function converts a path (list of x, y coordinates) to a list of
		# rotation and step values 
		
		# Convert the path's (x,y) coordinates to headings
		path_headings = []
		last_node = path[0]
		last_heading = 'up'
		for i in xrange(1,len(path)):
			current_node = path[i]
			current_heading = last_heading
			for move, heading in self.MOVEMENTS[1].iteritems():
				if (last_node[0] + move[0], last_node[1] + move[1]) == current_node:
					current_heading = heading
			last_heading = current_heading
			last_node = current_node	
			path_headings.append(current_heading)
		self.graph.optimal_path_headings = path_headings[:]

		# Optimize path by combining up to three forward steps into one move
		path_headings_and_steps = []	
		last_heading = 'up'
		count = 0	
		for i in range(len(path_headings)):
			if (path_headings[i] != last_heading) or (count == 3):
				path_headings_and_steps.append((last_heading, count))
				count = 0
				if i == len(path_headings)-1:
					path_headings_and_steps.append((path_headings[i], count + 1))						
			else:
				if i == len(path_headings)-1:
					path_headings_and_steps.append((path_headings[i], count + 1))					
			count += 1
			last_heading = path_headings[i]

		# Convert headings to rotations	
		path_rotations_and_steps = []
		last_heading = 'up'	
		for i in range(len(path_headings_and_steps)):
			rotation = self.HEADINGS[1][last_heading] - self.HEADINGS[1][path_headings_and_steps[i][0]]
			if rotation == 270:
				rotation = -90
			elif rotation == - 270:
				rotation = 90
			steps = path_headings_and_steps[i][1]
			path_rotations_and_steps.append((rotation, steps))
			last_heading = path_headings_and_steps[i][0]

		return path_rotations_and_steps

	def explore(self,sensors):
		# This function determines and returns a rotation and step value pair
		if self.graph.junctions and sum(sensors) == 0: # Turn right if in a dead end
			self.graph.dead_ends.append(self.location)
			return (90, 0)
		if (int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0)) == 2 \
		and self.last_location in self.graph.dead_ends: # Add to dead end list if applicable
			self.graph.dead_ends.append(self.location)
		# Initial priority value set depending on whether to use heuristic-first or visits-first 	
		heuristic_first = True
		top_priority = (0.5, self.maze_dim, 100, 1, 3 ) # (dead-end penalty, distance to target, visists, non-junction-penalty, number of steps)
		if self.location in self.graph.node_visits:
			if self.graph.node_visits[self.location] > 1: # Change to visits-first priority when current cell has been previously visited
				top_priority = (0.5, 100, 1, 3, self.maze_dim)
				heuristic_first = False

		target_node = (0,0) # When goal is found, robot will go back to the starting square
		# Add some sub-goals 
		if self.graph.goal_node==():
			if not self.graph.sub_goal in self.graph.node_visits:
				target_node = self.graph.sub_goal
			else:					
				target_node = (self.maze_dim / 2 - 0.5, self.maze_dim / 2 - 0.5)

		priority_moves = []		
		possible_moves = [(top_priority,(90,0)), (top_priority,(-90,0))] # Initial possible moves

		# Add more entries to possible moves list when available
		for i in range(len(self.STEERING_DIRECTIONS)):
			rotation = self.STEERING_DIRECTIONS[i] * self.ROTATION_ANGLE
			movement = 0
			distance = sensors[i] # Number of open squares in the sensor's direction
			new_heading = self.HEADINGS[0][(self.HEADINGS[1][self.heading]-rotation)]
			if distance > 3: # Limit number of steps to 3
				movement = 3
			else:
				movement = distance	
			if movement > 0:
				current_node = self.location[:]
				new_node = self.location[:]
				for steps in range(distance): # Used to add entries to the links list and prioritize possible moves
					new_node = (
						current_node[0] + self.MOVEMENTS[0][new_heading][0] , 
						current_node[1] + self.MOVEMENTS[0][new_heading][1] )
					self.graph.add_link(current_node, new_node)	# add link between current node and new node
					current_node = new_node	
					if (steps + 1) <= 3: # Add cells up to 3 squares away to list of possible move destinations
						if new_node in self.graph.node_visits: # cost is set based on number of visits 
							cost = self.graph.node_visits[new_node]
						else:
							cost = 0


						heuristic = abs(new_node[0] - target_node[0]) + abs(new_node[1] - target_node[1])
						#print "heuristic = ", heuristic, [new_node,target_node] 
						dead_end_penalty = int(new_node in self.graph.dead_ends)
						dead_end_penalty += int(new_node in self.graph.before_first_junction) * int(self.graph.goal_node==())
						dead_end_penalty += int(self.graph.goal_node!=() and new_node == self.graph.goal_node)
						non_junction_penalty = int(not new_node in self.graph.junctions)

						if heuristic_first:
							priority =	(dead_end_penalty, heuristic, cost, non_junction_penalty, steps + 1)	
						else:
							priority =	(dead_end_penalty, cost, non_junction_penalty, steps + 1, heuristic)
						if priority < top_priority:
							top_priority = priority
						
						#priority = (0,0,0,1,heuristic)
						#top_priority = priority
						possible_moves.append((priority, (rotation, steps + 1) ))

		for result in possible_moves:
			if result[0] == top_priority:
				priority_moves.append(result[1])

		# Randomly select rotation and movement from list of possible moves
		random_choice = np.random.randint( len( priority_moves ) )
		rotation, movement = priority_moves[random_choice]

		if not self.graph.junctions:
			if not self.last_location in self.graph.before_first_junction:
				self.graph.before_first_junction.append(self.last_location)

		# Check if this cell is a junction
		if (int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0)) < 2:
			if not self.location in self.graph.junctions:
				self.graph.junctions.append(self.location)

		if self.location in self.graph.dead_ends or not self.graph.junctions:
			movement = 1
		return (rotation, movement)

	def solve(self):
		return self.fast_run_moves[self.fast_run_step]	

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

		if self.exploring:
			if self.location in self.graph.node_visits:
				self.graph.node_visits[self.location] += 1
			else:
				self.graph.node_visits[self.location] = 1
			self.graph.display_grid(self.location, self.heading, self.maze_dim, "Steps:"+ str(self.graph.steps) +"\nVisited: " + str(len(self.graph.node_visits)) + "\nSub Goal" + str(self.graph.sub_goal) + "\nGoal Node:" + str(self.graph.goal_node) )
			if self.location in self.graph.goal_area:
				self.graph.goal_node = self.location[:]
				for node in self.graph.goal_area:
					if not node == self.graph.goal_node:
						self.graph.dead_ends.append(node)

				for node in self.graph.node_visits:
					self.graph.node_visits[node] = 0
				for node in self.graph.before_first_junction:
					self.graph.node_visits[node] = 0	
				self.graph.node_visits[self.graph.junctions[0]] = 0	
					
			if self.location == self.graph.start_node and self.graph.goal_node!=():
			#if len(self.graph.node_visits) + 3 == self.maze_dim * self.maze_dim:	
				self.exploring = False
				best_path = self.graph.best_path(self.graph.start_node, self.graph.goal_node)
				self.fast_run_moves = self.moves_to_goal(best_path)
				self.graph.display_grid(self.location, self.heading, self.maze_dim, "Steps:"+ str(self.graph.steps) +"\nVisited: " + str(len(self.graph.node_visits)) + "\nSub Goal:" + str(self.graph.sub_goal) + "\nGoal Node:" + str(self.graph.goal_node) + "\nPath length:" + str(len(self.graph.optimal_path_nodes)-1) +  "\nMoves:" + str(len(self.fast_run_moves)))
				#print self.graph.steps,",", str(((len(self.graph.node_visits) * 1.0) / self.maze_dim ** 2 ) * 100), ",",(self.graph.steps * 1.0)/30 + len(self.fast_run_moves), ",",len(self.graph.optimal_path_nodes)-1, ",", len(self.fast_run_moves)
				
				return ('Reset', 'Reset')
			rotation, movement = self.explore(sensors)		
		else:
			rotation, movement = self.solve()
			self.fast_run_step += 1

		self.last_location = self.location[:]		
		self.location, self.heading = self.next_location_and_heading (
			self.location, 
			self.heading, 
			rotation, 
			movement )		
		self.graph.steps+=1
		
		return rotation, movement