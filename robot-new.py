import numpy as np 
from Queue import PriorityQueue
import os
import time

class bcolors:
	# colors used is displaying paths
    BLUE = '\033[94m'
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
		self.dead_ends = [self.start_node]
		self.node_visits = {}
		self.optimal_path_nodes = []
		self.optimal_path_headings = []
		self.before_first_junction = ()
		self.steps = 0
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
		# Start of Djikstra's Search Algorithm
		frontier = PriorityQueue()
		frontier.put((0,node_1))
		from_node = {}
		from_node[node_1] = None
		cost = {}
		cost[node_1] = 0
		path = []
		current_node = ()
		while not frontier.empty():
			current_node = (frontier.get())[1]	
			if not (current_node in self.goal_area):
				for next_node in self.neighbors(current_node):
					last_node = current_node
					if from_node[current_node]:
						last_node = from_node[current_node] 
					turn_penalty = 1	
					if (last_node[0] + current_node[0] + next_node[0])/3 == next_node[0]:
						turn_penalty = 0
					new_cost = cost[current_node] + turn_penalty + 1
					if next_node not in cost or new_cost < cost[next_node]:
						cost[next_node] = new_cost
						priority = new_cost
						frontier.put((priority,next_node))
						from_node[next_node] = current_node
			else:
				break		
		path = [current_node]
		while current_node != self.start_node:
			current_node = from_node[current_node]
			path.append(current_node)
		path.reverse()
		# End of Djikstra's Search Algorithm
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
						col += bcolors.RED + " + " + bcolors.ENDC					
					elif (x,y) in self.node_visits:
						if (x,y) in (self.goal_area + self.optimal_path_nodes):
							char = " + "
							if (x,y) in self.optimal_path_nodes:
								idx = self.optimal_path_nodes.index((x,y))
								if idx == len(self.optimal_path_headings):
									idx -= 1								
								char = ARROWS[self.optimal_path_headings[idx]]		
							col += bcolors.GREEN + char + bcolors.ENDC
						elif (x,y) == self.before_first_junction:
							col += bcolors.BLUE + " + " + bcolors.ENDC		
						else:
							col += bcolors.YELLOW + " + " + bcolors.ENDC
					else:
						char = " + "
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
		
		path_headings = []
		# Convert the path's (x,y) coordinates to headings
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

		# Optimize path by converting up to three forward steps into one move
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
		if sum(sensors) == 0:
			self.graph.dead_ends.append(self.location)
			return (90, 0)
		if (int(sensors[0]==0) + int(sensors[1]==0) + int(sensors[2]==0)) == 2 \
		and self.last_location in self.graph.dead_ends:
			self.graph.dead_ends.append(self.location)
				
		possible_moves = []
		priority_moves = []
		top_priority = (1, 1000, self.maze_dim)
		# Add to possible moves 
		for i in range(len(self.STEERING_DIRECTIONS)):
			rotation = self.STEERING_DIRECTIONS[i] * self.ROTATION_ANGLE
			movement = 0
			distance = sensors[i]
			new_heading = self.HEADINGS[0][(self.HEADINGS[1][self.heading]-rotation)]
			if distance > 3:
				movement = 3
			else:
				movement = distance	
			if movement > 0:
				current_node = self.location[:]
				new_node = self.location[:]
				for steps in range(distance):
					new_node = (
						current_node[0] + self.MOVEMENTS[0][new_heading][0] , 
						current_node[1] + self.MOVEMENTS[0][new_heading][1] )
					self.graph.add_link(current_node, new_node)	
					current_node = new_node	
					if (steps + 1) <= 3:
						if new_node in self.graph.node_visits:
							cost = self.graph.node_visits[new_node]
						else:
							cost = 0
						center = self.maze_dim / 2 - 0.5
						heuristic = abs(new_node[0] - center) + abs(new_node[1] - center)
						dead_end_penalty = int(new_node in self.graph.dead_ends)
						priority =	(dead_end_penalty, cost, heuristic)
						if priority < top_priority:
							top_priority = priority
						possible_moves.append((priority, (rotation, steps + 1) ))
		#print possible_moves

		for result in possible_moves:
			if result[0] == top_priority:
				priority_moves.append(result[1])


		# Randomly select rotation and movement from list of possible moves
		random_choice = np.random.randint( len( priority_moves ) )
		rotation, movement = priority_moves[random_choice]
		if self.location in self.graph.dead_ends:
			movement = 1
		return (rotation, 1)
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
			self.graph.display_grid(self.location, self.heading, self.maze_dim, "Steps:"+ str(self.graph.steps))
			if self.location in self.graph.goal_area:
				self.goal_node = self.location[:]
				self.exploring = False
				best_path = self.graph.best_path(self.graph.start_node, self.graph.goal_node)
				self.fast_run_moves = self.moves_to_goal(best_path)
				self.graph.display_grid(self.location, self.heading, self.maze_dim, "Steps: " + str(self.graph.steps))
				# print "Exploration Steps:", self.graph.steps
				# print "Best path length:", len(self.graph.optimal_path_nodes)-1
				# print "Best path moves:", len(self.fast_run_moves)
				# print "Maze coverage:", str(((len(self.graph.node_visits) * 1.0) / self.maze_dim ** 2 ) * 100), "%"
				print self.graph.steps,",", str(((len(self.graph.node_visits) * 1.0) / self.maze_dim ** 2 ) * 100), ",",(self.graph.steps * 1.0)/30 + len(self.fast_run_moves), ",",len(self.graph.optimal_path_nodes)-1, ",", len(self.fast_run_moves)

				
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