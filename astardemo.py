#Assignment 5/Project
#A* 

import numpy as np
import matplotlib.pyplot as plt

import random

def node_collision(robot_rad, obs_coord, obs_rad, node_coord):

	for i in reversed(range(len(node_coord))):
		x_node = node_coord[i][0]
		y_node = node_coord[i][1]

		buffer_x_min = x_node - robot_rad
		buffer_x_max = x_node + robot_rad
		buffer_y_min = y_node - robot_rad
		buffer_y_max = y_node + robot_rad


		if buffer_x_min < 0 or buffer_x_max > 100 or buffer_y_min < 0 or buffer_y_max > 100:
			node_coord.pop(i)

		else: 
			for j in range(len(obs_coord)):
				min_dist = robot_rad + obs_rad[j]
				
				x_obs = obs_coord[j][0]
				y_obs = obs_coord[j][1]

				dist = ((x_node - x_obs)**2 + (y_node - y_obs)**2)**(0.5)

				if dist < min_dist:
					node_coord.pop(i)
					break

	return node_coord

def edge_collision(robot_rad, obs_coord, obs_rad, node1, node2):
	#checks if two nodes are in collision and will return 0 if they collide and the distance between the nodes if they don't collide

	x_node1 = node1[0]
	y_node1 = node1[1]

	x_node2 = node2[0]
	y_node2 = node2[1]

	#if node1 and node2 happen to be the same node
	if x_node1 == x_node2 and y_node1 == y_node2:
		return 0 

	for i in range(len(obs_coord)):
		x_obs = obs_coord[i][0]
		y_obs = obs_coord[i][1]

		if x_node1 == x_node2:
			#vertical line
			x_int = x_node1
			y_int = y_obs

		elif y_node1 == y_node2:
			#horizontal line
			x_int = x_obs
			y_int = y_node1

		else:
			#calculate equations of lines to find intersection
			m = 1.0*(y_node2 - y_node1)/(x_node2 - x_node1)
			b1 = y_node1 - m*x_node1
			b2 = y_obs + (1/m)*x_obs #slope of perpendicular line is (-1/m)
			x_int = (b2 - b1)/(m + (1/m))
			y_int = m*x_int + b1

		min_dist = robot_rad + obs_rad[i]

		#dist is the perpendicular distance from the center of the obstacle to the closest intersection point with the line
		dist = ((x_int - x_obs)**2 + (y_int - y_obs)**2)**.5

		#check if intersection point is between the two nodes
		dist1 = ((x_int - x_node1)**2 + (y_int - y_node1)**2)**.5
		dist2 = ((x_int - x_node2)**2 + (y_int - y_node2)**2)**.5
		distn = ((x_node1 - x_node2)**2 + (y_node1 - y_node2)**2)**.5

		if dist < min_dist and dist1 < distn and dist2 < distn:
			#Collision
			#print "Collision from",node1, "to", node2, "with obstactle at: ", obs_coord[i]
			return 0 

	return distn


def node_graph(robot_rad, obs_coord, obs_rad, node_coord):
	#creates a matrix of distances between collision free nodes

	g_size = len(node_coord) 
	graph = [[0 for col in range(g_size)] for row in range(g_size)]
	for i in range(g_size):
		node1 = node_coord[i]
		for j in range(g_size):			
			node2 = node_coord[j]
			graph[i][j] = edge_collision(robot_rad, obs_coord, obs_rad, node1, node2)
	return graph

def all_neighbors(graph, node_coord):
	#returns the coordinates of the neighbors for every node
	# stored as nx? where n is the same order as node_coord and ? will vary depending on how many neighbors each node has
	neighbors = []

	for i in range(len(node_coord)):
		neighbor_temp = []
		for j in range(len(node_coord)):
			if graph[i][j] != 0:
				neighbor_temp.append(node_coord[j])
		neighbors.append(neighbor_temp)
	return neighbors

def new_open_neighbors(current_neighbors, open_nodes, closed_nodes):
	for i in range(len(current_neighbors)):
		if current_neighbors[i] not in open_nodes and current_neighbors[i] not in closed_nodes:
			open_nodes.append(current_neighbors[i])

	return open_nodes

def heuristic_cost_to_go(node_neighbor, goal): 
	x_neighbor = node_neighbor[0]
	y_neighbor = node_neighbor[1]

	x_goal = goal[0]
	y_goal = goal[1]

	cost = (((x_neighbor - x_goal)**2) + ((y_neighbor - y_goal)**2))**.5
	return cost

def sort_open(open_nodes, node_coord, past_cost, goal):
	est_total_cost = []

	for i in range(len(open_nodes)):
		node_index = node_coord.index(open_nodes[i])
		est_total_cost.append((past_cost[node_index] + heuristic_cost_to_go(open_nodes[i],goal),open_nodes[i]))

	sort_cost = sorted(est_total_cost, key = lambda dist:dist[0])
	sorted_open = []
	#returns the coordinates of the sorted list 
	for i in range(len(open_nodes)):
		sorted_open.append(sort_cost[i][1])

	return sorted_open

##############################################
def plot_nodes(old_node_coord, node_coord, obs_coord, obs_rad, path_to_current):
	#prints all the things that are unavailble as magenta, the nodes that are available will be written over in another color
	
	x_unavailable = []
	y_unavailable =[]
	for i in range(len(old_node_coord)-2):
		x_unavailable.append(old_node_coord[i+1][0]) 
		y_unavailable.append(old_node_coord[i+1][1])

	x_start = old_node_coord[0][0]
	y_start = old_node_coord[0][1]

	x_goal = old_node_coord[len(old_node_coord)-1][0]
	y_goal = old_node_coord[len(old_node_coord)-1][1]
	

	plt.plot(x_unavailable,y_unavailable, 'b*')#################
	plt.plot(x_start, y_start, 'bD')###############
	plt.plot(x_goal, y_goal, 'bD')##################


	#obstacles
	for i in range(len(obs_coord)):
		cir = plt.Circle((obs_coord[i][0],obs_coord[i][1]), radius = obs_rad[i], fc = 'b')
		ax = plt.axes() #fixes ratio so cirles are circluar
		ax.add_patch(cir)


	#prints nodes that are available in blue, start and goal are red
	x = []
	y = []
	for i in range(len(node_coord)-2):
		x.append(node_coord[i+1][0]) 
		y.append(node_coord[i+1][1])

	x_start = old_node_coord[0][0]
	y_start = old_node_coord[0][1]

	x_goal = old_node_coord[len(old_node_coord)-1][0]
	y_goal = old_node_coord[len(old_node_coord)-1][1]

	plt.plot(x,y, 'c*')#######################
	plt.plot(x_start, y_start, 'rD')##########################
	plt.plot(x_goal, y_goal, 'rD')##########################

	#print the path in red
	for i in range(len(path_to_current)-1):
		plt.plot([path_to_current[i][0],path_to_current[i+1][0]],[path_to_current[i][1],path_to_current[i+1][1]], 'r')######################

	plt.axis('equal')
	plt.axis([0, 100,0,100])
	plt.show()


def Astar_complete(robot_rad, obs_coord, obs_rad, node_coord, start, goal):

	#start at beginning and goal at end
	node_coord.insert(0,start)
	node_coord.append(goal)

	new_node_coord = node_coord[:] #so that the original node_coord remains unchanged

	#removes nodes that would result in collision with an obstacle or the 100x100 planar boundary
	new_node_coord = node_collision(robot_rad, obs_coord, obs_rad, new_node_coord)


	if start not in new_node_coord or goal not in new_node_coord:
		print "start or goal conflicts with an obstacle"
		plot_nodes(node_coord, new_node_coord, obs_coord, obs_rad, [])
		return

	graph = node_graph(robot_rad, obs_coord, obs_rad, new_node_coord)

	possible_neighbors = all_neighbors(graph, new_node_coord)

	num_nodes = len(new_node_coord) #just to make things a little shorter later on

	##Initializations

	open_nodes = [start]
	path_to_current = []
	
	#parent will end up storing the index of the parent node
	parent = [0]*num_nodes #num_nodes+1 was used so that there is no valid reference until filled in

	#past_cost is initialized with 0 for the start and inf for everything else
	past_cost = float('inf')*np.ones(num_nodes)
	past_cost[0] = 0

	closed_nodes = []

	while len(open_nodes) != 0:

		#current stores the coordinates of the current node
		current = open_nodes.pop(0)

		#find node index
		current_index = new_node_coord.index(current)

		#add current to list of closed nodes
		closed_nodes.append(current)

		if current == goal:
			print "Path to goal"
			path_to_current = [goal]
			node_index = new_node_coord.index(goal)
			while path_to_current[0] != start:
				path_to_current.insert(0,parent[node_index])
				node_index = new_node_coord.index(path_to_current[0])
			print path_to_current
			plot_nodes(node_coord, new_node_coord, obs_coord, obs_rad, path_to_current)

			return


		current_neighbors = possible_neighbors[current_index]
		open_nodes = new_open_neighbors(current_neighbors, open_nodes, closed_nodes)

		# #OPEN now has added all the current neighbors
		for nbr in current_neighbors:
			nbr_index = new_node_coord.index(nbr)
		 	tentative_past_cost = past_cost[current_index] + graph[current_index][nbr_index]
			if tentative_past_cost < past_cost[nbr_index]:
				past_cost[nbr_index] = tentative_past_cost
				parent[nbr_index] = current
				open_nodes = sort_open(open_nodes, new_node_coord, past_cost, goal)


	if len(path_to_current) == 0:
		print "No possible path"
		plot_nodes(node_coord, new_node_coord, obs_coord, obs_rad, path_to_current)
		return


def random_inputs(num_nodes, num_obs):
	
	robot_rad = 2*random.random()

	obs_coord = []
	obs_rad = []

	for i in range(num_obs):
		obs_coord.append([100*random.random(),100*random.random()])
		obs_rad.append(10*random.random())
	node_coord = []

	for i in range(num_nodes):
		node_coord.append([100*random.random(),100*random.random()])

	# start = [100*random.random(),100*random.random()]
	# goal = [100*random.random(),100*random.random()]
	start = [5,5]
	goal = [90,90]

	return robot_rad, obs_coord, obs_rad, node_coord, start, goal
	
	
def Astar_only(graph, node_coord, start, goal):
	
	# #start at beginning and goal at end
	# node_coord.insert(0,start)
	# node_coord.append(goal)

	new_node_coord = node_coord[:]

	if start not in new_node_coord or goal not in new_node_coord:
		print "start or goal conflicts with an obstacle"
		return []

	possible_neighbors = all_neighbors(graph, new_node_coord)

	num_nodes = len(new_node_coord) #just to make things a little shorter later on

	##Initializations

	open_nodes = [start]
	path_to_current = []
	
	#parent will end up storing the index of the parent node
	parent = [0]*num_nodes #num_nodes+1 was used so that there is no valid reference until filled in

	#past_cost is initialized with 0 for the start and inf for everything else
	past_cost = float('inf')*np.ones(num_nodes)
	past_cost[0] = 0

	closed_nodes = []

	while len(open_nodes) != 0:

		#current stores the coordinates of the current node
		current = open_nodes.pop(0)

		#find node index
		current_index = new_node_coord.index(current)

		#add current to list of closed nodes
		closed_nodes.append(current)

		if current == goal:
			print "Path to goal"
			path_to_current = [goal]
			node_index = new_node_coord.index(goal)
			while path_to_current[0] != start:
				path_to_current.insert(0,parent[node_index])
				node_index = new_node_coord.index(path_to_current[0])
			return path_to_current


		current_neighbors = possible_neighbors[current_index]
		open_nodes = new_open_neighbors(current_neighbors, open_nodes, closed_nodes)

		# #OPEN now has added all the current neighbors
		for nbr in current_neighbors:
			nbr_index = new_node_coord.index(nbr)
		 	tentative_past_cost = past_cost[current_index] + graph[current_index][nbr_index]
			if tentative_past_cost < past_cost[nbr_index]:
				past_cost[nbr_index] = tentative_past_cost
				parent[nbr_index] = current
				open_nodes = sort_open(open_nodes, new_node_coord, past_cost, goal)

	if len(path_to_current) == 0:
		print "No possible path"
		return []	

#end of code

#### testing with fixed inputs
robot_rad = 2
obs_coord = [[30,30],[37,62],[37,75],[50,62], [62,50]]
obs_rad = [4,6,5,3,4]
node_coord = [[25,25], [25,50],[50,25], [50,50], [75,25], [75, 50], [75,75]]
start = [25,75]
goal = [50,75]

print ""
print "Astar (fixed inputs)"
a = Astar_complete(robot_rad, obs_coord, obs_rad, node_coord, start, goal)


#### testing with random inputs
f = random_inputs(10,5)

robot_rad = f[0]
obs_coord = f[1]
obs_rad = f[2]
node_coord = f[3]
start = f[4]
goal = f[5]

node_coord.insert(0,start)
node_coord.append(goal)

new_node_coord = node_collision(robot_rad, obs_coord, obs_rad, node_coord)
graph = node_graph(robot_rad, obs_coord, obs_rad, node_coord)

print ""
print "Astar (random inputs)"
h = Astar_complete(f[0], f[1], f[2], f[3], f[4], f[5])










