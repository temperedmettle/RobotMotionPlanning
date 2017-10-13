from Queue import PriorityQueue
import numpy as np
import os
import time

class bcolors:
	# colors used is displaying paths
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Graph:
	# Base class for the graph that we will be using to implement nodes and links
	def __init__(self, rows, cols):
		self.nodes = []
		self.links = []
		# Initialize the nodes in a grid
		for x in range(cols):
			for y in range (rows):
				self.nodes.append((x,y))

	def neighbors(self, node):
		# Return neighboring nodes
		directions = [(1,0),(0,1),(-1,0),(0,-1)]
		result = []
		for direction in directions:
			neighbor = (node[0] + direction[0], node[1] + direction[1])
			if neighbor in self.nodes:
				result.append(neighbor)						
		return result
				
class MicromouseGraph(Graph):
	# This is a child class of Graph. Micromouse-specific properties and functions
	# are implemented here.
	def __init__(self, size):
		Graph.__init__(self, size, size)		
		self.goal_explored = False # Will be True, the first time the robot reaches the goal
		self.HEADINGS = [
			{'up':90, 'down':270, 'left':180, 'right':0},
			{90:'up', 270:'down', 180:'left', 0:'right'}
		]
		self.MOVEMENTS = {'up':(0,1),'down':(0,-1),'left':(-1,0),'right':(1,0)}
		self.TURNS = {'left':90, 'none':0, 'right':-90}
		self.start_node = (0,0)
		self.before_first_junction = ()
		self.grid_size = size
		self.goal_nodes = [
			((size / 2),(size / 2) ),
			((size / 2)-1,(size / 2)-1), 
			((size / 2),(size / 2)-1),
			((size / 2)-1,(size / 2))
			]
		self.node_visits = {} # This will be used to record the number of visits in a node
		self.dead_ends = [] # This will beused to record dead end paths
		self.optimal_path = [] # This will contain rotation and movement values 
		self.optimal_path_nodes = [] # This will contain nodes
		self.optimal_path_headings = [] # This will contain headings


	def display_grid(self, location, heading, notes):
		# This function will display the path taken by the robot, as well as the optimal path
		# that the robot has found. The following color coding is used:
		#
		# 		yellow	= 	visited
		# 		red 	= 	dead end
		#		green 	= 	goal node or optimal path node
		#		black 	= 	the robot
		#
		# Additionally, the robot and optimal paths will include heading information 

		time.sleep(0.05)
		ARROWS = {'up':' ^ ','right':' > ','down':' v ','left':' < '}
		os.system('cls' if os.name == 'nt' else 'clear')
		for y in range(self.grid_size - 1, -1 , -1):
			col = "  "
			for x in range(self.grid_size):	
				if (x,y) == location:
					col += bcolors.BOLD + ARROWS[heading] + bcolors.ENDC
				else:
					if (x,y) in self.dead_ends:
						col += bcolors.FAIL + " + " + bcolors.ENDC					
					elif (x,y) in self.node_visits:
						if (x,y) in (self.goal_nodes + self.optimal_path_nodes):
							char = " + "
							if (x,y) in self.optimal_path_nodes:
								idx = self.optimal_path_nodes.index((x,y))
								if idx == len(self.optimal_path_headings):
									idx -= 1								
								char = ARROWS[self.optimal_path_headings[idx]]		
							col += bcolors.OKGREEN + char + bcolors.ENDC
						elif (x,y) == self.before_first_junction:
							col += bcolors.OKBLUE + " + " + bcolors.ENDC		
						else:
							col += bcolors.WARNING + " + " + bcolors.ENDC
					else:
						char = " + "
						if (x,y) in self.optimal_path_nodes:
							idx = self.optimal_path_nodes.index((x,y))
							if idx == len(self.optimal_path_headings):
								idx -= 1
							char = ARROWS[self.optimal_path_headings[idx]]								
							col += bcolors.OKGREEN + char + bcolors.ENDC
						else:	
							col += "   "		
			print col
		print notes	

	def remove_link(self,node_1,node_2):
		if (node_1,node_2) in self.links:
			self.links.remove((node_1,node_2))
		if (node_2,node_1) in self.links:
			self.links.remove((node_2,node_1))		

	def add_link(self,node_1,node_2):
		if not (node_1,node_2) in self.links:
			self.links.append((node_1,node_2))
		if not (node_2,node_1) in self.links:
			self.links.append((node_2,node_1))	

	def add_visit(self, node):
		if node in self.node_visits:
			self.node_visits[node] += 1
		else:
			self.node_visits[node] = 1				

	def update_links(self, location, heading, sensors): # Add new links based on sensor readings
		MOVEMENT = {'up':(0,1),'down':(0,-1),'left':(-1,0),'right':(1,0)}
		TURNS = [self.TURNS['left'],self.TURNS['none'],self.TURNS['right']] 
		for i in range(len(sensors)):	
			reading = sensors[i]
			new_heading = self.HEADINGS[1][(self.HEADINGS[0][heading] + TURNS[i]) % 360]
			if reading > 0:
				current_node = location[:]
				new_node = location[:]
				for j in range(reading):
					new_node = (current_node[0] + self.MOVEMENTS[new_heading][0], current_node[1] + self.MOVEMENTS[new_heading][1])
					self.add_link(current_node, new_node)		
					current_node = new_node								

	def remove_node(self,node):
		if node in self.nodes:
			neighbors = self.neighbors(node)
			for neighbor in neighbors:
				if (node, neighbor) in self.links:
					self.remove_link(node, neighbor)
			self.nodes.remove(node)

	def heading(self, node, next_node): # Computes the heading given two nodes
		HEADINGS = {(0,1):'up', (1,0):'right', (0,-1):'down', (-1,0):'left'}
		delta = (next_node[0] - node[0], next_node[1] - node[1])
		return HEADINGS[delta]

	def cost(self,node_1,node_2,node_3): # Computes the cost given three nodes
		last_heading = 'up'
		if node_1:
			last_heading = self.heading(node_1, node_2)
		new_heading = self.heading(node_2, node_3)
		if last_heading == new_heading: # Give a lower cost if the robot does not turn
			return 1
		else:
			return 2

	def set_goal_entry_node(self, location): # This prevents the robot from spending more time in the goal area
		for node in self.goal_nodes:
			if (not node == location) and (node in self.nodes):
				self.nodes.remove(node)

	def next_node(self, current_node, heading, turn, steps):
		# Returns the next node given current node, its current heading, the turn and the number of steps
		movement = self.MOVEMENTS[
			self.HEADINGS[1][
				(self.HEADINGS[0][heading] + self.TURNS[turn]) % 360
			]
		]
		next_node = (current_node[0] + (movement[0] * steps), current_node[1] + (movement[1] * steps))
		if next_node in self.nodes:
			return next_node
		else:
			return None	

	def next_exploration_move(self, node, heading):
		neighbors = self.neighbors(node)
		top_priority = 100000 # A high number for making just a right turn
		prioritized_results = [(
			top_priority,
			(90,0),
			node, 
			self.HEADINGS[1][(self.HEADINGS[0][heading] - 90)%360]
		)] # Turn to the right in case there's nowhere else to go.
		for key, value in self.TURNS.iteritems():
			next_node = self.next_node(node, heading, key, 1)
			if next_node:
				if (node, next_node) in self.links: # Check if path to next_node is open.
					cost = 0
					if next_node in self.dead_ends:
						cost = 200000 # Prevent getting into dead end paths
					elif next_node == self.before_first_junction and not self.goal_explored:
							cost = 200000 # Prevent the robot from reaching the starting point when goal has not been explored
					elif next_node in self.node_visits:	
						if next_node in self.goal_nodes:	
							cost = 200000 # Prevent the robot from re-entering the goal area during exploration
						else:
							cost = self.node_visits[next_node] # Use visit count to avoid loops
					if self.goal_explored:
						heuristic = self.heuristic(next_node, 'origin') # Use heuristic to get to the start node faster.
					else:
						if (0,15) in self.node_visits:
							heuristic = self.heuristic(next_node, 'center') # Use heuristic to get to the goal faster.
						else:	
							heuristic = self.heuristic(next_node, 'top-left') # Use heuristic to get to the top-right corner.
					priority = cost * 100 + heuristic # prioritize by visits then heuristic
					if priority < top_priority:
						top_priority = priority
					prioritized_results.append((
						priority, 
						(-value,1), 
						next_node, 
						self.HEADINGS[1][(self.HEADINGS[0][heading] + value) % 360]
					))
		# Select the best result based on priority (randomly if priority is the same)			
		results_left = []
		for result_item in prioritized_results:
			if result_item[0] == top_priority:
				results_left.append(result_item)	
		best_result = ()	
		pick = 0
		if len(results_left) > 1:
			pick = np.random.randint(0,len(results_left)-1)
		best_result = results_left[pick][-3:]	
		return best_result	

	def heuristic(self,node,endpoint): # Use distance to center for heuristic
		if endpoint == 'center':
			center = (self.grid_size / 2) - 1.5
			return abs(node[0] - center) + abs(node[1] - center)
		elif endpoint == 'origin':	
			return abs(node[0] - 0) + abs(node[1] - 0)
		elif endpoint == 'top-right':
			return abs(node[0] - 15) + abs(node[1] - 15)
		else: 	
			return abs(node[0] - 0) + abs(node[1] - 15)
	def search_Astar(self): # A* implementation of path finder
		frontier = PriorityQueue()
		frontier.put((0,self.start_node))
		from_node = {}
		from_node[self.start_node] = None
		cost = {}
		cost[self.start_node] = 0
		path = []
		current_node = ()
		while not frontier.empty():
			current_node = (frontier.get())[1]	
			if not (current_node in self.goal_nodes):
				for next_node in self.neighbors(current_node):
					if (current_node,next_node) in self.links:
						new_cost = cost[current_node] + self.cost(from_node[current_node], current_node, next_node)
						if next_node not in cost or new_cost < cost[next_node]:
							cost[next_node] = new_cost
							priority = new_cost + self.heuristic(next_node, 'center')
							frontier.put((priority,next_node))
							from_node[next_node] = current_node
			else:
				break
		path = [current_node]
		while current_node != self.start_node:
			current_node = from_node[current_node]
			path.append(current_node)
		path.reverse()	
		self.optimal_path_nodes = path
		return path

	def get_path(self, search_type): # Returns optimal path (in rotations and movements)
		path = []
		path_headings = []
		DIRECTIONS = [
			{'up':(0,1),'down':(0,-1),'left':(-1,0),'right':(1,0)},
			{(0,1):'up',(0,-1):'down',(-1,0):'left',(1,0):'right'},
			{'up':90, 'down':270, 'left':180, 'right':0}
		]
		if search_type == 'A*':
			path = self.search_Astar() # Get optimal path using A*

		# Convert the path's (x,y) coordinates to headings
		last_node = path[0]
		last_heading = 'up'
		for i in xrange(1,len(path)):
			current_node = path[i]
			current_heading = last_heading
			for move, heading in DIRECTIONS[1].iteritems():
				if (last_node[0] + move[0], last_node[1] + move[1]) == current_node:
					current_heading = heading
			last_heading = current_heading
			last_node = current_node	
			path_headings.append(current_heading)

		# Optimize path by converting up to three forward steps into one move
		path_heading_steps = []	
		last_heading = 'up'
		count = 0	
		for i in range(len(path_headings)):
			if (path_headings[i] != last_heading) or (count == 3):
				path_heading_steps.append((last_heading, count))
				count = 0
				if i == len(path_headings)-1:
					path_heading_steps.append((path_headings[i], count + 1))						
			else:
				if i == len(path_headings)-1:
					path_heading_steps.append((path_headings[i], count + 1))					
			count += 1
			last_heading = path_headings[i]

		# Convert headings to rotations	
		path_rotation_steps = []
		last_heading = 'up'	
		for i in range(len(path_heading_steps)):
			rotation = DIRECTIONS[2][path_heading_steps[i][0]] - DIRECTIONS[2][last_heading]
			if rotation == 270:
				rotation = -90
			elif rotation == - 270:
				rotation = 90
			steps = path_heading_steps[i][1]
			path_rotation_steps.append((-rotation, steps))
			last_heading = path_heading_steps[i][0]	
		self.optimal_path_headings = path_headings	
		return path_rotation_steps

