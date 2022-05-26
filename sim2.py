import random, os, math
import numpy as np
import config
import block_occupation
import routing
from copy import deepcopy

def boundary_check(system, x, y, w, h):
	# w and h here includes microbump overhead
	if (x - w / 2) < 0:
		return False
	if (x + w / 2) > system.intp_size:
		return False
	if (y - h / 2) < 0:
		return False
	if (y + h / 2) > system.intp_size:
		return False
	return True

def close_neighbor(system, grid):
	''' slightly moving chiplets, do not consider rotation'''
	chiplet_order = np.random.permutation(range(system.chiplet_count))
	granularity = system.granularity
	for p in chiplet_order:
		direction_order = np.random.permutation(['up', 'down', 'left', 'right'])
		xx, yy, width, height = system.x[p], system.y[p], system.width[p] + 2 * system.hubump[p], system.height[p] + 2 * system.hubump[p]
		# print ('chiplet ', p + 2)
		for d in direction_order:
			# print (d)
			# re-connect the direction with the appropriate function in order to easily visulize using print-grid(). The dirctions are referring to the grid printed on screen, the directions in functions are referring to conventional x-y coordinates, origin in the left-bottom corner.
			if d == 'left':
				if block_occupation.check_down_occupation(grid, granularity, xx, yy - granularity, width, height):
					# print ('chiplet', p + 2, d)
					return p, xx, yy - granularity
			elif (d == 'right') and (yy + granularity <= system.intp_size):
				if block_occupation.check_up_occupation(grid, granularity, xx, yy + granularity, width, height):
					# print ('chiplet', p + 2, d)
					return p, xx, yy + granularity
			elif d == 'up':
				if block_occupation.check_left_occupation(grid, granularity, xx - granularity, yy, width, height):
					# print ('chiplet', p + 2, d)
					return p, xx - granularity, yy
			elif (d == 'down') and (xx + granularity <= system.intp_size):
				if block_occupation.check_right_occupation(grid, granularity, xx + granularity, yy, width, height):
					# print ('chiplet', p + 2, d)
					return p, xx + granularity, yy
	print ('No chiplet can be moved.')
	exit()

def sort_area(system):
	n = system.chiplet_count
	area = [None] * system.chiplet_count
	for i in range(0,n):
		area[i] = system.height[i]*system.width[i]
	rank = [index for index, value in sorted(list(enumerate(area)), key=lambda x: x[1])]
	return rank

def jumping_neighbor(system, grid):
	'''define a neighbor placement as move one chiplet to anywhere can be located.
	rotate if needed. We do not consider swapping, since can't gaurantee the placement
	is still legal (no overlap) after swapping'''

	n = system.chiplet_count
	granularity = system.granularity
	rank = sort_area(system)
	for i in range(0,n):
		count = 0
		while True:
			pick_chiplet = rank[n-i-1]
			x_new = random.randint(1, system.intp_size / granularity - 1) * granularity
			y_new = random.randint(1, system.intp_size / granularity - 1) * granularity
			rotation = random.randint(0,1)
			if rotation == 1:
				chiplet_width, chiplet_height = system.height[pick_chiplet], system.width[pick_chiplet]
			else:
				chiplet_height, chiplet_width = system.height[pick_chiplet], system.width[pick_chiplet]
			if boundary_check(system, x_new, y_new, chiplet_width + 2 * system.hubump[pick_chiplet], chiplet_height + 2 * system.hubump[pick_chiplet]) and block_occupation.replace_block_occupation(grid, granularity, x_new, y_new, chiplet_width + 2 * system.hubump[pick_chiplet], chiplet_height + 2 * system.hubump[pick_chiplet], pick_chiplet):
				print ('found a random placement at', count, 'th trial')
				return pick_chiplet, x_new, y_new, rotation
			count += 1
			if count > 2000:
				# it's not easy to find a legal placement using random method. try move each chiplet (in random order) slightly until find a legal solution
				print ('cannot find a legal random placement, go with close_neighbor')
				break
	return close_neighbor(system, grid)

def accept_probability(old_length, new_length, T, weight):
	b = 1
	if length_min != length_max:
		old_cost = b * (old_length - length_max) / (length_max - length_min)
		new_cost = b * (new_length - length_max) / (length_max - length_min)
	else:
		old_cost = b * (old_length - length_max)
		new_cost = b * (new_length - length_max)
	delta = - (new_cost - old_cost)
	if delta > 0:
		ap = 1
	else:
		ap = math.exp(delta / T)
	return ap, new_cost, old_cost

# def accept_probability(old_temp, new_temp, T):
# 	delta = (old_temp - new_temp)
# 	# print (old_temp, new_temp, T, delta)
# 	if delta > 0:
# 		ap = 1
# 	else:
# 		ap = math.exp( delta / T )
# 	return ap

def update_minmax(length):
	global length_max, length_min
	if length > length_max:
		length_max = length
	if length < length_min:
		length_min = length

def register_log(system_best, step_best, length_best, T, step):
	with open(system_best.path + 'log.txt', 'a+') as LOG:
		LOG.write('T = ' +str(T)+'\t step = '+str(step)+ '\n')
		LOG.write(str(step_best) + '\n'  + str(length_best) + '\n')
		LOG.write(str(system_best.x)+'\n'+str(system_best.y) + '\n')

def register_step(system, step,  length, T):
	with open(system.path + 'step.txt', 'a+') as LOG:
		LOG.write('T = ' +str(T)+'\t step = '+str(step)+ '\n')
		LOG.write(str(length) + '\n')
		LOG.write(str(system.x)+'\n'+str(system.y) + '\n')
		LOG.write(str(system.width)+'\n' + str(system.height)+'\n')

def anneal():
	# first step: read config and generate initial placement
	global  length_max, length_min
	length_max, length_min =  0, 200
	system = config.read_config()
	system_new = deepcopy(system)
	system_best = deepcopy(system)
	step = 1
	step_best = 1
	length_current = routing.solve_Cplex(system)
	length_best =  length_current
	update_minmax( length_current)
	print ('step_'+str(step), 'length =', length_current)
	x_best, y_best = system.x[:], system.y[:]
	intp_size = system.intp_size
	granularity = system.granularity
	grid = block_occupation.initialize_grid(int(intp_size/granularity))
	for i in range(system.chiplet_count):
		grid = block_occupation.set_block_occupation(grid, granularity, system.x[i], system.y[i], system.width[i] + 2 * system.hubump[i], system.height[i] + 2 * system.hubump[i], i)
	# block_occupation.print_grid(grid)

	# set annealing parameters
	T = 1.0
	T_min = 0.01
	alpha = system.decay
	# jumping_ratio = T_min / alpha
	jumping_ratio = 0.9 # fixed to 10% chance to jump
	# start simulated annealing
	register_log(system_best, step_best, length_best, T, step)
	register_step(system, step, length_current, T)
	while T > T_min:
		i = 1
		while i <= intp_size:
			step += 1
			print ('step_'+str(step), ' T = ',T, ' i = ', i)
			jump_or_close = random.random()
			if 1 - jumping_ratio > jump_or_close:
				chiplet_moving, x_new, y_new, rotation = jumping_neighbor(system, grid)
			else:
				chiplet_moving, x_new, y_new = close_neighbor(system, grid)
				rotation = 0
			print ('moving chiplet', chiplet_moving + 2, 'from (', system.x[chiplet_moving], system.y[chiplet_moving], ') to (', x_new, y_new, '), rotation = ', rotation)
			system_new = deepcopy(system)
			system_new.x[chiplet_moving], system_new.y[chiplet_moving] = x_new, y_new
			if rotation == 1:
				system_new.rotate(chiplet_moving)
				# system_new.height[chiplet_moving], system_new.width[chiplet_moving] = system_new.width[chiplet_moving], system_new.height[chiplet_moving]
			length_new = routing.solve_Cplex(system_new)
			print('Length =', length_new)
			print('cost=',)
			update_minmax(length_new)
			register_step(system_new, step, length_new, T)
			# ap = accept_probability(temp_current, temp_new, T)
			ap, new_cost, old_cost = accept_probability(length_current, length_new, T, system.weight)
			r = random.random()
			if ap > r:
				# clear last step's occupation of chiplet_moving (system)
				grid = block_occupation.clear_block_occupation(grid, granularity, system.x[chiplet_moving], system.y[chiplet_moving], system.width[chiplet_moving] + 2 * system.hubump[chiplet_moving], system.height[chiplet_moving] + 2 * system.hubump[chiplet_moving], chiplet_moving)
				# set new occupation with rotation (system_new)
				grid = block_occupation.set_block_occupation(grid, granularity, x_new, y_new, system_new.width[chiplet_moving] + 2 * system.hubump[chiplet_moving], system_new.height[chiplet_moving] + 2 * system.hubump[chiplet_moving], chiplet_moving)
				# update system
				system = deepcopy(system_new)
				length_current = length_new
				bap, cur_cost, best_cost = accept_probability(length_best, length_current, T, system.weight)
				if bap >= 1:
					length_best = length_new
					system_best = deepcopy(system_new)
					step_best = step
				print ('AP = ', ap, ' > ', r, ' Accept!', 'cost',new_cost)
				# block_occupation.print_grid(grid)
			else:
				print ('AP = ', ap, ' < ', r, ' Reject!', 'cost', old_cost)
			i += 1
		register_log(system_best, step_best, length_best, T, step)
		T *= alpha
		# jumping_ratio /= alpha
	os.system('rm ' + system.path + '{*.flp,*.lcf,*.ptrace,*.steady}')
	# os.system('gs -dBATCH -dNOPAUSE -q -sDEVICE=pdfwrite -sOutputFile='+system.path+'combine.pdf '+system.path + 'step_{1..'+str(step_best)+'}L4.pdf')
	os.system('rm ' + system.path + 'step_*.pdf')
	return system_best, step_best, length_best

if __name__ == "__main__":
	solution, step_best,length_best = anneal()
	print ('final solution: step, temp')
	print (step_best)
	print (length_best)
	print (solution.x)
	print (solution.y)
	with open(solution.path+'output.txt','w') as OUTPUT:
		OUTPUT.write(str(step_best) + '\n' + str(length_best) + '\n')
		OUTPUT.write(str(solution.x)+ '\n' + str(solution.y) + '\n')


