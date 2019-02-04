# ACT (anti-Collision Test) framework
import numpy as np, matplotlib.pyplot as pl

# To be done:
# logs: fail/success (collision/goal), if success how many steps, decision on ego control for each step, nb hard breaking, TTC = safety
# forall scenarios, compile all the stats above
# output % collisions, mean time to goal when SAFE, Hard breaking
# function model: input: state; output: control
# function ego: position + goal, control = [-2,-1,0,+1], hard breaking = -2

# Parameters
p = {'iter': 0,
		 'n_obj_left': 5,
		 'n_obj_right': 5,
		 'numframes': 100,
		 'timeStep':0.2}
p['n_obj'] = p['n_obj_left']+p['n_obj_right']

def sc0_init(): # init scenario 0
	data = []
	# init ego and objs
	if 0: # obj ahead of ego
		[data.append(100), data.append(0), data.append(0), data.append(20)]
		[data.append(100), data.append(100), data.append(0), data.append(10)]
		p['n_obj_left'], p['n_obj_right'] = 1, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	if 0: # obj behind ego
		[data.append(100), data.append(100), data.append(0), data.append(10)]
		[data.append(100), data.append(0), data.append(0), data.append(20)]
		p['n_obj_left'], p['n_obj_right'] = 1, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	if 0: # 2 objs ahead of ego
		[data.append(100), data.append(0), data.append(0), data.append(20)]
		[data.append(100), data.append(100), data.append(0), data.append(10)]
		[data.append(100), data.append(200), data.append(0), data.append(0)]
		p['n_obj_left'], p['n_obj_right'] = 2, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	if 0: # 1 obj ahead of ego, 1 behind
		[data.append(100), data.append(100), data.append(0), data.append(10)]
		[data.append(100), data.append(0), data.append(0), data.append(20)]
		[data.append(100), data.append(200), data.append(0), data.append(0)]
		p['n_obj_left'], p['n_obj_right'] = 2, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	if 0: # 1 obj ahead of ego, 1 behind, in diagonal
		[data.append(100), data.append(100), data.append(-10/np.sqrt(2)), data.append(10/np.sqrt(2))]
		[data.append(200), data.append(0), data.append(-20/np.sqrt(2)), data.append(20/np.sqrt(2))]
		[data.append(0), data.append(200), data.append(-30/np.sqrt(2)), data.append(30/np.sqrt(2))]
		p['n_obj_left'], p['n_obj_right'] = 2, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	if 1: # 1 obj ahead of 2 objs, 1 behind, in diagonal
		[data.append(100), data.append(  0), data.append(-10/np.sqrt(2)), data.append(10/np.sqrt(2))] # ego
		[data.append(200), data.append(100), data.append( 0), data.append(0)] # obj1
		[data.append(100), data.append(100), data.append( 0), data.append(0)] # obj2
		[data.append(  0), data.append(100), data.append(10), data.append(0)] # obj3
		p['n_obj_left'], p['n_obj_right'] = 3, 0
		p['n_obj'] = p['n_obj_left'] + p['n_obj_right']
	
	return data

def sc1_init(): # init scenario 1
	data = []
	# init ego and objs
	[data.append(100), data.append(0), data.append(0), data.append(20)]  # ego
	for i in np.arange(p['n_obj_left']):  # objects left
		data.append(np.random.uniform(0, 50))  # x
		data.append(np.random.uniform(25, 190))  # y
		data.append(np.random.uniform(10, 25))  # vx
		data.append(np.random.uniform(0, 0))  # vy
	for i in np.arange(p['n_obj_right']):  # objects right
		data.append(np.random.uniform(150, 200))  # x
		data.append(np.random.uniform(25, 190))  # y
		data.append(-np.random.uniform(10, 25))  # vx
		data.append(-np.random.uniform(0, 0))  # vy
	return data
	
def sc1_step(state_vector):
	control_vector = [0, 0]
	return control_vector

def idm_step(obj, s, delta_v):
	x, y, vx, vy = obj
	v = np.sqrt(vx**2+vy**2)
	
	# model IDM, assumes same parameters forall objs in the scene
	params = {'v0': 10, # desired velocity v0: the velocity the vehicle would drive at in free traffic
						's0': 0, # minimum spacing s0: a minimum desired net distance. A car can't move if the distance from the car in the front is not at least s0
						'T': 0,  # desired time headway T: the minimum possible time to the vehicle in front
						'a': 2,  # acceleration a: the maximum vehicle acceleration
						'b': 4}  # comfortable braking deceleration  b: a positive number
	
	axy = 1 - (v / params['v0']) ** 4  # free road
	# s_star = params['s0'] + v * params['T'] + v * delta_v / (2 * np.sqrt(params['a'] * params['b']))
	s_star = params['s0'] + v * params['T'] + v * delta_v / (2 * np.sqrt(params['a'] * params['b']))
	axy = axy - (s_star / s) ** 2  # interaction term
	axy = params['a'] * axy
	
	return max(min(axy, params['a']), -params['b'])
	
# Mechanic transition model Ts*s + Ta*a
def transition_ego(state): # state = [x,y,vx,vy], control = [ax,ay]
	Ts, Ta = transition(p['timeStep']) # time step
	return np.array(Ts)@np.array(state) + np.array(Ta)@np.array([0,0])

def transition_obj(state, control):
	Ts, Ta = transition(p['timeStep'])  # time step
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
				[ dt,           0.0],
				[0.0,            dt]]
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

def isOnLine(curr_obj, data):
	ax, ay, bx, by = curr_obj[0:4] # obj considered
	
	id, idx = [], 0
	for i in np.arange(1 + p['n_obj']):
		if i == 0:
			do_nothing = True # ego
		elif curr_obj == data[idx:idx + 4]:
			do_nothing = True  # skip considered obj for comparison
		else: # obj
			cx, cy, _, _ = data[idx:idx + 4] # obj
			ab = [bx, by]
			ac = [cx-ax, cy-ay]
			ab[0], ab[1] = ab[0] / np.sqrt(ab[0] ** 2 + ab[1] ** 2), ab[1] / np.sqrt(ab[0] ** 2 + ab[1] ** 2)
			ac[0], ac[1] = ac[0] / np.sqrt(ac[0] ** 2 + ac[1] ** 2), ac[1] / np.sqrt(ac[0] ** 2 + ac[1] ** 2)
			dot_prod = ab[0]*ac[0]+ab[1]*ac[1]
			if dot_prod == np.sqrt(ab[0]**2+ab[1]**2)*np.sqrt(ac[0]**2+ac[1]**2): # obj is ahead of ego (based on v_ego)
				id.append(i)
			if dot_prod == -np.sqrt(ab[0]**2+ab[1]**2)*np.sqrt(ac[0]**2+ac[1]**2): # obj is behind ego (based on v_ego)
				do_nothing = True
		idx += 4
		
	return id

def statsAheadOnLine(curr_obj, data, id):
	x, y, vx, vy = curr_obj
	d, delta_v = np.inf, 0 # delta_v leads to free-space in IDM
	for i in id:
		xo, yo, vxo, vyo = data[i*4:i*4 + 4]
		di = np.sqrt((x-xo)**2+(y-yo)**2)
		if di < d:
			d = di # closest vehicle ahead on line
			delta_v = np.sqrt(vxo**2+vyo**2) - np.sqrt(vx**2+vy**2) # difference of longi speed (on line ~longi)
	
	return d, delta_v

def png2gif(interval):
	import os, imageio
	filenames = sorted([x for x in os.listdir('./figs') if x.endswith(".png")], # over all png files ...
										 key=lambda p: os.stat('./figs/'+p).st_mtime) # ... re-order by increasing file creation date
	
	images = []
	for filename in filenames:
		images.append(imageio.imread('./figs/' + filename))
	imageio.mimsave('./figs/iterations.gif', images, duration=interval)


def plot_setup(axs, iter):
	pl.cla()
	axs.set_xlim([0, 600]), axs.set_ylim([0, 200])  # grid = 200x200
	pl.xlabel('timestep_' + str(iter))

def plot_scatter(data):
	pl.scatter(data[0], data[1], marker='o', c='blue')  # ego
	pl.scatter(data[4:4 * (1 + p['n_obj_left']):4], data[5:4 * (1 + p['n_obj_left']) + 1:4],
						 marker='o', c='red')  # objs left
	pl.scatter(data[4 * (1 + p['n_obj_left'])::4], data[4 * (1 + p['n_obj_left']) + 1::4],
						 marker='o', c='green')  # objs right
	
def plot_annotate(data):
	idx = 0
	for _ in np.arange(1 + p['n_obj']):
		x, y, vx, vy = data[idx:idx + 4]
		pl.annotate('v=' + str(np.sqrt(vx ** 2 + vy ** 2)) + 'm/s', xy=(x + 2, y))
		idx += 4
	# pl.annotate('v=' + str(np.sqrt(data[2] ** 2 + data[3] ** 2)) + 'm/s', xy=(data[0] + 2, data[1]))  # ego

def plot_quiver(axs, data):
	axs.quiver(data[0], data[1], data[2], data[3], color='b',
						 width=5e-3, scale_units='xy', angles='xy', scale=1)  # ego
	axs.quiver(data[4:4 * (1 + p['n_obj_left']):4], data[5:4 * (1 + p['n_obj_left']) + 1:4],
						 data[6:4 * (1 + p['n_obj_left']) + 2:4], data[7:4 * (1 + p['n_obj_left']) + 3:4],color='r',
						 width=5e-3, scale_units='xy', angles='xy', scale=1)  # objs left
	axs.quiver(data[4 * (1 + p['n_obj_left'])::4], data[4 * (1 + p['n_obj_left']) + 1::4],
						 data[4 * (1 + p['n_obj_left']) + 2::4], data[4 * (1 + p['n_obj_left']) + 3::4], color='g',
						 width=5e-3, scale_units='xy', angles='xy', scale=1)  # objs right
	# quiver doc, cf http://matplotlib.1069221.n5.nabble.com/scaling-arrows-in-quiver-td23758.html
	
def main():
	# Init plot
	fig, axs = pl.subplots(1, 1)
	
	data = sc0_init()
	# data = sc1_init()
	for iter in np.arange(0, p['numframes']):
		if data[0]<0 or data[0]>200 or data[1]<0 or data[1]>200: # early termination
			break
		
		# Plots
		plot_setup(axs, iter)
		plot_scatter(data)
		plot_annotate(data)
		plot_quiver(axs, data)
		pl.savefig('./figs/timestep_'+str(iter)+'.png') # save figure as png, replaces pl.show()
		
		# stats as prerequisite for IDM
		idx = 0
		controls = []
		for _ in np.arange(1 + p['n_obj']):
			if _ == 0:
				# data[0:4] = transition_ego(data[0:4])
				do_nothing = True
				controls.append(np.array([np.nan, np.nan]))
			else:
				if 0: # transition model
					data[idx:idx + 4] = transition_obj(data[idx:idx + 4])  # obj
				else: # idm
					curr_obj = data[idx:idx + 4]
					id = isOnLine(curr_obj, data)
					s, delta_v = statsAheadOnLine(curr_obj, data, id)  # returns distance, delta_v of closest obj ahead
					axy = idm_step(data[idx:idx + 4], s, -delta_v) # delta_v = v_alpha - v_alpham1, where alpham1 is vehicle in front
					
					u = np.array(data[idx+2:idx+4]) # unit vector along speed vector
					if u[0]**2+u[1]**2 != 0:
						u = u/np.sqrt((u[0]**2+u[1]**2))
					control = axy * u
					controls.append(control) # controls should be computed before any change is applied to other objs
					
					# data[idx:idx + 4] = transition_obj(data[idx:idx + 4], control)
			idx += 4
		
		idx = 0
		for i in np.arange(1 + p['n_obj']):
			if i == 0:
				data[0:4] = transition_ego(data[0:4])
			else:
				data[idx:idx + 4] = transition_obj(data[idx:idx + 4], controls[i])
			idx += 4
	
	# save sequence of .png to .gif
	png2gif(0.1)

main()