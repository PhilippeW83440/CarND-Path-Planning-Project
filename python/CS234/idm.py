# Depict IDM behavior (free + brake)
import numpy as np, matplotlib.pyplot as pl

# model IDM, assumes same parameters forall objs in the scene
params = {'v0': 10,# desired velocity v0: the velocity the vehicle would drive at in free traffic
					's0': 0, # minimum spacing s0: a minimum desired net distance. A car can't move if the distance from the car in the front is not at least s0
					'T': 0,  # desired time headway T: the minimum possible time to the vehicle in front
					'a': 2,  # acceleration a: the maximum vehicle acceleration
					'b': 4}  # comfortable braking deceleration  b: a positive number

def idm_step(veh_alpha, veh_alpham1):
	a_free = params['a']*(1-(veh_alpha['sdot']/params['v0'])**4) # free
	
	delta_sdot = veh_alpha['sdot'] - veh_alpham1['sdot']
	s_star = params['s0'] + veh_alpha['sdot'] * params['T'] + veh_alpha['sdot'] * delta_sdot / (2 * np.sqrt(params['a'] * params['b']))
	a_brake = -params['a'] * (s_star / (veh_alpham1['s']-veh_alpha['s'])) ** 2 # brake
	
	veh_alpha['sddot'] = a_free + a_brake
	
	return max(min(veh_alpha['sddot'], params['a']), -params['b']), veh_alpha['sdot'], delta_sdot, veh_alpham1['s']-veh_alpha['s']
	
# Mechanic transition model Ts*s + Ta*a
def transition_obj(state, control):
	Ts, Ta = transition(0.1) # time step
	return np.array(Ts)@np.array(state) + np.array(Ta)@np.array(control)

def transition(dt):
	# s=[s,sdot]
	Ts = [[1.0, 0.0,  dt, 0.0],
				[0.0, 1.0, 0.0,  dt],
				[0.0, 0.0, 1.0, 0.0],
				[0.0, 0.0, 0.0, 1.0]]
	
	# a = sddot
	Ta = [[0.5 * dt ** 2, 0.0],
				[0.0, 0.5 * dt ** 2],
				[ dt,           0.0],
				[0.0,            dt]]
	return Ts, Ta

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
	# -------------------- Check IDM free only --------------------
	veh_alpha = {'s': 0, 'sdot': 0, 'sddot': 0}
	veh_alpham1 = {'s': 2**32, 'sdot': 0, 'sddot': 0} # veh_alpham1['s'] == np.inf => free only
	s_h, sd_h, sdd_h, t_h = [], [], [], []
	
	t = 0
	for i in np.arange(100):
			s, sd, sdd = veh_alpha['s'], veh_alpha['sdot'], veh_alpha['sddot']
			sdd, _, _, _ = idm_step(veh_alpha, veh_alpham1)
			veh_alpha['s'], _, veh_alpha['sdot'], _ = transition_obj([s, 0, sd, 0], [sdd, 0])
			
			s, sd, sdd = veh_alpham1['s'], veh_alpham1['sdot'], veh_alpham1['sddot']
			veh_alpham1['s'], _, veh_alpham1['sdot'], _ = transition_obj([s, 0, sd, 0], [sdd, 0])
			
			s_h.append(veh_alpha['s'])
			sd_h.append(veh_alpha['sdot'])
			sdd_h.append(veh_alpha['sddot'])
			t += 0.1
			t_h.append(t)
	
	# Plot
	fig, axs = pl.subplots(1, 1)
	axs.plot(t_h, s_h)
	axs.plot(t_h, sd_h)
	axs.plot(t_h, sdd_h)
	axs.legend(['s', 'sd', 'sdd'])
	pl.title('IDM free only')
	pl.xlabel('t')
	pl.ylabel('vehicle alpha')
	pl.show(block=False)
	
	# -------------------- Check IDM brake only --------------------
	params['v0'] = 10
	veh_alpha = {'s': 0, 'sdot': 10, 'sddot': 0}
	veh_alpham1 = {'s': 100, 'sdot': 0, 'sddot': 0}  # veh_alpham1['s'] == np.inf => brake only
	s_h, sd_h, sdd_h, t_h = [], [], [], []
	v_h, delta_v_h, d_h = [], [], []
	
	t = 0
	for i in np.arange(200):
		s, sd, sdd = veh_alpha['s'], veh_alpha['sdot'], veh_alpha['sddot']
		sdd, v, delta_v, d = idm_step(veh_alpha, veh_alpham1)
		v_h.append(v), delta_v_h.append(delta_v), d_h.append(d) # debug purpose
		veh_alpha['s'], _, veh_alpha['sdot'], _ = transition_obj([s, 0, sd, 0], [sdd, 0])
		veh_alpha['sddot'] = sdd
		
		s, sd, sdd = veh_alpham1['s'], veh_alpham1['sdot'], veh_alpham1['sddot']
		veh_alpham1['s'], _, veh_alpham1['sdot'], _ = transition_obj([s, 0, sd, 0], [sdd, 0])
		
		s_h.append(veh_alpha['s'])
		sd_h.append(veh_alpha['sdot'])
		sdd_h.append(veh_alpha['sddot'])
		t += 0.1
		t_h.append(t)
	
	# Plot
	fig, axs = pl.subplots(1, 1)
	axs.plot(t_h, s_h)
	axs.plot(t_h, sd_h)
	axs.plot(t_h, sdd_h)
	axs.plot(t_h, v_h, '--x')
	axs.plot(t_h, delta_v_h, '--x')
	axs.plot(t_h, d_h, '--x')
	axs.legend(['s', 'sd', 'sdd', 'v', 'delta_v', 'd'])
	pl.title('IDM brake+free')
	pl.xlabel('t')
	pl.ylabel('vehicle alpha')
	pl.show(block=True)

main()