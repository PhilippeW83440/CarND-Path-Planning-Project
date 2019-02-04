# ACT (anti-Collision Test) framework
import numpy as np, sys
import matplotlib.pyplot as pl
from matplotlib.animation import FuncAnimation

# Parameters
p = {'iter': 1,
		 'n_obj_left': 5,
		 'n_obj_right': 5,
		 'numframes': 100}
p['n_obj'] = p['n_obj_left']+p['n_obj_right']

# function model: input: state; output: control
# function ego: position + goal, control = [-2,-1,0,+1]
# hard breaking = -2

def sc1_init(): # init scenario 1
	ego = {'state': {'x': 100, 'y': 0, 'vx': 0, 'vy': 20}, 'control': {'ax': 0, 'ay': 0}}
	print(ego['state']['x'])
	
	goal = {'state': {'x': ego['state']['x'], 'y': 200, 'vx': 0, 'vy': 0}}
	
	obj = {'state':
					 {'x': 0 + 0.5 * np.random.randn(p['n_obj']),
						'y': 0 + 0.5 * np.random.randn(p['n_obj']),
						'vx': 0 + 0.5 * np.random.randn(p['n_obj']),
						'vy': 0 + 0.5 * np.random.randn(p['n_obj'])},
				 'control':
					 {'ax': 0 + 0.5 * np.random.randn(p['n_obj']),
						'ay': 0 + 0.5 * np.random.randn(p['n_obj'])}}
	obj_id = 0
	print(obj['state']['x'][obj_id])
	
	state_vector = []
	state_vector.append(ego['state']['x'])
	state_vector.append(ego['state']['y'])
	state_vector.append(ego['state']['vx'])
	state_vector.append(ego['state']['vy'])
	for i in range(p['n_obj']):
		state_vector.append(obj['state']['x'][obj_id])
		state_vector.append(obj['state']['y'][obj_id])
		state_vector.append(obj['state']['vx'][obj_id])
		state_vector.append(obj['state']['vy'][obj_id])
	
	return state_vector
	
def sc1_step(state_vector):
	control_vector = [0, 0]
	return control_vector

def idm_step(state):
	x, y, vx, vy = state
	# model IDM
	v0 = 5  # desired velocity v0: the velocity the vehicle would drive at in free traffic
	s0 = 0  # minimum spacing s0: a minimum desired net distance. A car can't move if the distance from the car in the front is not at least s0
	T = 1  # desired time headway T: the minimum possible time to the vehicle in front
	a = 2  # acceleration a: the maximum vehicle acceleration
	b = 4  # comfortable braking deceleration  b: a positive number
	ax = 1 - (vx / v0) ** 4  # free road
	v, delta_v = np.sqrt(vx**2+vy**2), np.nan
	s_star = s0 + v * T + v * delta_v / (2 * np.sqrt(a * b))
	dist = 0  # distance with car ahead
	ax = ax - (s_star / dist) ** 2  # interaction term
	ax = a * ax
	
	control = [0,0]
	return control
	
# Mechanic transition model Ts*s + Ta*a
def transition_ego(state): # state = [x,y,vx,vy], control = [ax,ay]
	Ts, Ta = transition(0.2) # time step
	return np.array(Ts)@np.array(state) + np.array(Ta)@np.array([0,0])

def transition_obj(state):
	Ts, Ta = transition(0.2)  # time step
	control = idm_step(state)
	return np.array(Ts)@np.array(state) + np.array(Ta)@np.array(control)

def transition(dt):
	# s=[x,y,vx,vy]
	Ts = [[1.0, 0.0,  dt, 0.0],
				[0.0, 1.0, 0.0,  dt],
				[0.0, 0.0, 1.0, 0.0],
				[0.0, 0.0, 0.0, 1.0]]
	
	# a = [ax, ay]
	Ta = [[0.5 * dt ** 2, 0.0],
				[0.0, 0.5 * dt ** 2],
				[dt, 0.0],
				[0.0, dt]]
	return Ts, Ta

# Not verified yet
def xy2sd(x0, y0, x1, y1): # assumes straight lines (d==0)
	t = [x1 - x0, y1 - y0]
	t = [ti/(t[0]**2+t[1]**2) for ti in t]
	
	s = np.sqrt(x1**2+y1**2)
	d = 0
	return s, d
	
# Not verified yet
def sd2xy(s, d, f_x, f_dx, f_y, f_dy): # assumes straight lines
	x = f_x(s) + d * f_dx(s)
	y = f_y(s) + d * f_dy(s)
	return x, y

def ttc(ego, obj, radius):
	x1, y1, vx1, vy1 = ego[0], ego[1], ego[2], ego[3]
	x2, y2, vx2, vy2 = obj[0], obj[1], obj[2], obj[3]
	
	a = (vx1 - vx2) **2 + (vy1 - vy2) **2
	b = 2 * ((x1 - x2) * (vx1 - vx2) + (y1 - y2) * (vy1 - vy2))
	c = (x1 - x2) **2 + (y1 - y2) **2 - radius **2
	
	if a == 0 and b == 0:
		if c == 0:
			return 0
		else:
			return np.inf

	if a == 0 and b != 0:
		t = -c / b
		if t < 0:
			return np.inf
		else:
			return t
	
	delta = b **2 - 4 * a * c
	if delta < 0:
		return np.inf
	
	t1 = (-b - np.sqrt(delta)) / (2 * a)
	t2 = (-b + np.sqrt(delta)) / (2 * a)
	if t1 < 0:
		t1 = np.inf
	
	if t2 < 0:
		t2 = np.inf
	
	return min(t1, t2)

def min_ttc(state_vector):
	radius = 15.0
	ego = state_vector[0:4]
	
	smallest_TTC = np.inf
	smallest_TTC_obj = -1
	
	idx = 4
	for id in np.arange(p['n_obj']):
		obj = state_vector[idx:idx+4]
		TTC = ttc(ego, obj, radius)
	
		if TTC < smallest_TTC:
			smallest_TTC = TTC
			smallest_TTC_obj = id
		idx += 4

	return smallest_TTC, smallest_TTC_obj
	
# sc1_step(): increment, return ax,ay

# logs: fail/success (collision/goal), if success how many steps, decision on ego control for each step, nb hard breaking, TTC = safety

# forall scenarios, compile all the stats above
# output % collisions, mean time to goal when SAFE, Hard breaking

def init_anim():
	fig, axs = pl.subplots(1, 1, squeeze=False, figsize=(5, 5))
	axs[0, 0].set_xlim([0, 200]), axs[0, 0].set_ylim([0, 200])  # grid = 200x200
	scat = axs[0, 0].scatter([], [], marker='o')
	# ann = axs[0, 0].annotate('ego', xy=(0,0)) # axs[0, 0].annotate('ego', xy=(x,y), xytext=(x-0.3,y-0.3), arrowprops = {'arrowstyle': "->"})
	arr, = axs[0, 0].plot([0, 0], [0, 0], 'r-', linewidth=1) # ~quiver
	return fig, axs[0, 0], scat, arr
def update_anim(i, data, ax, scat, arr):
	pl.xlabel('timestep {0}'.format(i))
	print(min_ttc(data))
	
	# Transitions
	data[0:4] = transition_ego(data[0:4])
		
	idx = 4
	for _ in np.arange(p['n_obj']):
		obj = data[idx:idx + 4]
		data[idx:idx + 4] = transition_obj(data[idx:idx + 4])  # obj
		idx += 4
	
	x,y,vx,vy = data[0::4],data[1::4],data[2::4],data[3::4]
	scat.set_offsets(np.c_[x, y])
	
	# ann.set_position(([xi+0.3 for xi in x], [yi+0.3 for yi in y]))
	# x1,x2,y1,y2 = x, [sum(s) for s in zip(x, vx)], y, [sum(s) for s in zip(y, vy)]
	# arr.set_data(np.c_[x1, x2], np.c_[y1, y2]) # NOK at the moment, vectors are connected
	# arr.set_data(np.c_[x1[0], x2[0]], np.c_[y1[0], y2[0]])
	
	return data

def main():
	fig, ax, scat, arr = init_anim()
	
	data = []
	# init ego and objs
	[data.append(100), data.append(0), data.append(0), data.append(20)]	# ego
	for i in np.arange(p['n_obj_left']): # objects left
		data.append(np.random.uniform(0,50))  # x
		data.append(np.random.uniform(25,190))# y
		data.append(np.random.uniform(10,25)) # vx
		data.append(np.random.uniform(0,5))   # vy
	for i in np.arange(p['n_obj_right']): # objects right
		data.append(np.random.uniform(150, 200))# x
		data.append(np.random.uniform(25, 190)) # y
		data.append(-np.random.uniform(10, 25)) # vx
		data.append(-np.random.uniform(0, 5))   # vy
	
	anim = FuncAnimation(fig, update_anim, frames=np.arange(0, p['numframes']), interval=20, repeat=False,
											 fargs=(data, ax, scat, arr))
	# anim.save('result.gif', dpi=80, writer='imagemagick')
	# Image(url='result.gif')
	pl.show()
	

main()