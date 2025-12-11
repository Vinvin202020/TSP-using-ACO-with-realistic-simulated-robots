#%% Necessary imports
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.optimize import root

#%% Micro modeling with turning
def run_micro_sim_turning(N0, dt, p_obs, p_pat, p_turn, T_obs, T_turn, num_iter= 200, seed= None):
	state_mapping= {"goto": 0, "obs": 1, "wait": 2, "turn": 3}
	rng= np.random.default_rng(seed)
	robots_states= np.zeros(N0)
	robots_turning_times= np.zeros(N0)
	counts_activ= np.empty(num_iter+1)
	counts_obs= np.empty(num_iter+1)
	counts_wait= np.empty(num_iter+1)
	counts_activ[0]= N0
	counts_obs[0]= 0
	counts_wait[0]= 0
	patrols_reached= 0
	time= 0
	for k in range(num_iter): # Number of time step to simulate
		time+= dt
		for i in range(N0): # State evolution loop
			if robots_states[i] == state_mapping["goto"]:
				rv= rng.uniform(0,1)
				if (rv < p_obs):
					robots_states[i]= state_mapping["obs"]
				elif (rv < p_obs + p_pat):
					rv= rng.uniform(0,1)
					if (rv < p_turn):
						robots_states[i]= state_mapping["turn"]
						robots_turning_times[i]= time
					else:
						robots_states[i]= state_mapping["wait"]
						patrols_reached+=1
			elif robots_states[i] == state_mapping["obs"]:
				rv= rng.uniform(0,1)
				if (rv < 1.0/T_obs):
					robots_states[i]= state_mapping["goto"]
			elif robots_states[i] == state_mapping["turn"]:
				if (time - robots_turning_times[i] >= T_turn):
					robots_states[i]= state_mapping["wait"]
					patrols_reached+=1
		counts_activ[k+1]= np.sum(robots_states == state_mapping["goto"])
		counts_obs[k+1]= np.sum(robots_states == state_mapping["obs"])
		counts_wait[k+1]= np.sum(robots_states == state_mapping["wait"])
		if counts_wait[k+1] == N0:
			robots_states= np.zeros(N0)
	return counts_activ, counts_obs, counts_wait, patrols_reached

def run(world_to_sim, num_iter= 200):
	A_arena= 16
	A_obs= 0.1*0.3
	A_rob= np.pi*(0.04*0.04)
	A_pat= np.pi*(0.02*0.02)
	wb_worlds_mapping= {"small": 0, "big": 1}
	if wb_worlds_mapping[world_to_sim]:
		N0= 10 # Number of robots to simulate
		p_obs= (10*A_obs + N0*A_rob + 9*A_pat)/A_arena # (total area of obstacles + robots)/(total area of arena)
		p_pat= 0.2 # 1/(average number of steps to reach patrol in webots (if no interference))
		p_turn= 0.01 # (number of times robots turned)/(number of patrols reached)
		T_obs= 5 # (average number of steps lost in obstacle avoidance)
		T_turn= 4.22 # (time of 1 revolution for turning robots in webots)
		dt= 0.032 # Picked to match webots timesteps
	else:
		# Measured values in webots
		N0= 5 # Number of robots to simulate
		p_obs= (5*A_obs + N0*A_rob + 4*A_pat)/A_arena # (total area of obstacles + robots + patrols - 1)/(total area of arena)
		p_turn= 0.1 # (number of times robots turned)/(number of patrols reached)
		T_obs= 44 # (average number of steps lost in obstacle avoidance)
		T_turn= 4.3 # (time of 1 revolution for turning robots in webots (in seconds))
		dt= 0.032 # Picked to match webots timesteps
		p_pat= 1/390.2
		#p_pat= 1/(800 - T_obs*800*p_obs - p_turn*T_turn/dt) # 1/(average number of steps to reach patrol in webots (if no obstacles and turning))
	print("1/p_pat for these settings: ", 1/p_pat)
	print("p_obs: ", p_obs)
	return run_micro_sim_turning(N0, dt, p_obs, p_pat, p_turn, T_obs, T_turn, num_iter)

world_to_sim= "small" # "small" or "big"
num_iter= 200000
counts_activ, counts_obs, counts_wait, pat_reached= run(world_to_sim, num_iter)
#%%
dt= 0.032
if world_to_sim == "small":
	N0= 5
else:
	N0= 10
num_to_plot= 200
mask = np.linspace(0, num_iter, num_to_plot, dtype=int)
times= np.linspace(0, num_iter*dt, num_iter+1)[mask]
fig= plt.figure()
counts_turn= N0 - counts_activ - counts_obs - counts_wait
plt.plot(times, counts_activ[mask], label= "active")
plt.plot(times, counts_obs[mask], label= "obs")
plt.plot(times, counts_wait[mask], label= "wait")
plt.plot(times, counts_turn[mask], label= "turn")
plt.ylabel("Counts")
plt.xlabel("$t$ [s]")
plt.grid()
plt.legend()
plt.show()

print("Average patrol reached per second per robots: ", round(pat_reached/(N0*num_iter*dt), 4))
print("Mean number of active robots: ", round(np.mean(counts_activ), 3))
print("Mean number of avoiding robots: ", round(np.mean(counts_obs), 3))
print("Mean number of waiting robots: ", round(np.mean(counts_wait), 3))
print("Mean number of turning robots: ", round(np.mean(counts_turn), 3))


#%% Macro-modelling (steady-state)
def run_macro_sim(N0, dt, p_obs, p_pat, p_turn, T_obs, T_turn, alpha, num_iter= 200):
	state_occupancy= np.empty(shape=(num_iter + 1, 4))
	state_occupancy[0, :]= np.array([0, 0, 0, N0])
	for i in range(1,num_iter + 1):
		state_occupancy[i, 0]= (state_occupancy[i-1, 0]*(1- np.exp(-alpha*(N0 - state_occupancy[i-1, 0])))
						 	   + p_pat*(1-p_turn)*state_occupancy[i-1, 3] + p_pat*p_turn*state_occupancy[max([0, int(np.floor(i - T_turn/dt))]), 2])
		state_occupancy[i, 1]= (state_occupancy[i-1, 1]*(1-1/T_obs) 
						 	   + p_obs*state_occupancy[i-1, 3])
		state_occupancy[i, 2]= (state_occupancy[i-1, 2] 
								+ p_pat*p_turn*(state_occupancy[i-1, 3] - state_occupancy[max([0, int(np.floor(i - T_turn/dt))]), 2]))
		
		state_occupancy[i, 0]= min(N0, state_occupancy[i, 0])
		state_occupancy[i, 0]= max(0, state_occupancy[i, 0])

		state_occupancy[i, 1]= min(N0, state_occupancy[i, 1])
		state_occupancy[i, 1]= max(0, state_occupancy[i, 1])

		state_occupancy[i, 2]= min(N0, state_occupancy[i, 2])
		state_occupancy[i, 2]= max(0, state_occupancy[i, 2])

		state_occupancy[i, 3]= (N0 - state_occupancy[i, 2]
								-state_occupancy[i, 1] - state_occupancy[i, 0])
	return state_occupancy

A_arena= 16
A_obs= 0.1*0.3
A_rob= np.pi*(0.04*0.04)
A_pat= np.pi*(0.02*0.02)
N0= 5 # Number of robots to simulate
p_obs= (5*A_obs + N0*A_rob + 4*A_pat)/A_arena # (total area of obstacles + robots + patrols - 1)/(total area of arena)
p_turn= 0.1 # (number of times robots turned)/(number of patrols reached)
T_obs= 44 # (average number of steps lost in obstacle avoidance)
T_turn= 4.3 # (time of 1 revolution for turning robots in webots)
dt= 0.032 # Picked to match webots timesteps
p_pat= 1/390.2
alpha=3.2
num_iter_macro= 200000
state_occupancy= run_macro_sim(N0, dt, p_obs, p_pat, p_turn, T_obs, T_turn, alpha, num_iter=num_iter_macro)

#%% Plotting macro-sim
N_w= state_occupancy[:, 0]
N_o= state_occupancy[:, 1]
N_t= state_occupancy[:, 2]
N_p= state_occupancy[:, 3]

num_to_plot= 200
mask = np.linspace(0, num_iter_macro, num_to_plot, dtype=int)
times= np.linspace(0, num_iter_macro*dt, num_iter_macro+1)[mask]
fig= plt.figure()
plt.plot(times, N_p[mask], label= "active")
plt.plot(times, N_o[mask], label= "obs")
plt.plot(times, N_w[mask], label= "wait")
plt.plot(times, N_t[mask], label= "turn")
plt.ylabel("Counts")
plt.xlabel("$t$ [s]")
plt.grid()
plt.legend()
plt.show()

print("Steady number of active robots: ", round(N_p[-1], 3))
print("Steady number of avoiding robots: ", round(N_o[-1], 3))
print("Steady number of waiting robots: ", round(N_w[-1], 3))
print("Steady number of turning robots: ", round(N_t[-1], 3))
# %%
