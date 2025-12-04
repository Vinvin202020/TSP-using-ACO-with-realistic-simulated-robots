#%% Necessary imports
import numpy as np
import matplotlib.pyplot as plt

#%% Micro-modelling
def run_micro_sim(N0, p_obs, p_pat, T_obs, num_iter= 200, seed= None):
	# State mapping: goto=0, obs= 1, wait= 2
	rng= np.random.default_rng(seed)
	robots_states= np.zeros(N0)
	counts_activ= np.empty(num_iter+1)
	counts_obs= np.empty(num_iter+1)
	counts_activ[0]= N0
	counts_obs[0]= 0
	partols_reached= 0
	for k in range(num_iter): # Number of time step to simulate
		for i in range(N0): # State evolution loop
			if robots_states[i] == 0:
				rv= rng.uniform(0,1)
				if (rv < p_obs):
					robots_states[i]= 1
				elif (rv < p_obs + p_pat):
					robots_states[i]= 2
					partols_reached+=1
			elif robots_states[i] == 1:
				rv= rng.uniform(0,1)
				if (rv < 1.0/T_obs):
					robots_states[i]= 0
		counts_activ[k+1]= np.sum(robots_states == 0)
		counts_obs[k+1]= np.sum(robots_states == 1)
		if (counts_activ[k+1] == 0) and (counts_obs[k+1] == 0):
			robots_states= np.zeros(N0)
	return counts_activ, counts_obs, partols_reached

N0= 2000
p_obs= 0.05
p_pat= 0.7
T_obs= 5
T_w= 5
dt= 0.1
num_iter= 200
counts_activ, counts_obs, pat_reached= run_micro_sim(N0, p_obs, p_pat, T_obs, num_iter)

#%%
times= np.linspace(0, num_iter*dt, num_iter+1)
fig= plt.figure()
plt.plot(times, counts_activ, label= "active")
plt.plot(times, counts_obs, label= "obs")
plt.plot(times, N0 - counts_activ - counts_obs, label= "wait")
plt.ylabel("Counts")
plt.xlabel("$t$ [s]")
plt.grid()
plt.legend()
plt.show()

print("Ave_pat_reached per steps per robots: ", round(pat_reached/(N0*(num_iter+1)), 4))
# %%
