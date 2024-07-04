# This script was used to create the plots in Section V.A, Computational
# cost and air utilization. The output of this script are 4 plots:
# - Computational cost with infinite range
# - Computational cost with limited range
# - Air utilization with infinite range
# - Air utilization with limited range
#
# The input to this script should be logs placed at 
# <script_path>/data/comp_time_air_utilization
# 
# Each log should be in a dedicated folder that contains
# a dedicated log file for each drone in the simulation named
# "log_drone_X.csv". The name of the folder should contain the
# sequence of letters "inf" (for infinite range) or "lim"
# (for limited range)
#



import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

import pandas as pd
import numpy as np
from plot_settings import *

save_figures = True

estimators = ['ref', 'dec', 'ful', 'dyn']
names = {'ref': 'Centralized EKF',
        'dec': 'Decoupled EKF',
        'ful': 'Full State EKF',
        'dyn': 'Dynamic EKF'}

# Organize logs by type and number of drones
data_dir = os.path.join(os.path.dirname(__file__), 'data/comp_time_air_utilization')    
log_directories = [ f.path for f in os.scandir(data_dir) if f.is_dir() ]
log_type = ['N/A' for _ in log_directories]
log_ndrones = [0 for _ in log_directories]

for i_log, log_path in enumerate(log_directories):
    _, name = os.path.split(log_path)
    if 'inf' in name:
        log_type[i_log] = 'inf'
    elif 'lim' in name:
        log_type[i_log] = 'lim'
    count = 0
    for file in os.listdir(log_path):
        if file.startswith('log_drone_') and "relpos" not in file:
            count += 1
    log_ndrones[i_log] = count

n_drones = np.unique(log_ndrones)

# aggregate data from all logs
all_air_utilization_inf = [np.array([]) for _ in n_drones]
# all_air_utilization_lim = [np.array([]) for _ in n_drones]
all_comp_time_inf = dict()
# all_comp_time_lim = dict()
for e in estimators:
    all_comp_time_inf[e] = [np.array([]) for _ in n_drones]
    # all_comp_time_lim[e] = [np.array([]) for _ in n_drones]

for i_log, log_path in enumerate(log_directories):
    # idx = n_drones.index(log_ndrones[i_log])
    idx = np.where(n_drones==log_ndrones[i_log])[0][0]
    for file in os.listdir(log_path):
        if "relpos" in file:
            continue
        if file.startswith('log_drone_'):
            name, _ = file.split('.')
            data = pd.read_csv(os.path.join(log_path, file), skipinitialspace=True)

            if log_type[i_log] == 'inf':
                all_air_utilization_inf[idx] = np.hstack((all_air_utilization_inf[idx], 100*data["uwb_load"].to_numpy()))
                for e in estimators:
                    all_comp_time_inf[e][idx] = np.hstack((all_comp_time_inf[e][idx], data[e+"_t_us"].to_numpy()))
            
            # elif log_type[i_log] == 'lim':
            #     all_air_utilization_lim[idx] = np.hstack((all_air_utilization_lim[idx], 100*data["uwb_load"].to_numpy()))
            #     for e in estimators:
            #         all_comp_time_lim[e][idx] = np.hstack((all_comp_time_lim[e][idx], data[e+"_t_us"].to_numpy()))
            

t_mean_inf = dict()
t_stdev_inf = dict()
t_q1_inf = dict()
t_median_inf = dict()
t_q3_inf = dict()
# t_mean_lim = dict()
# t_stdev_lim = dict()

air_mean_inf = np.array([0.0 for _ in n_drones])
air_stdev_inf = np.array([0.0 for _ in n_drones])
# air_mean_lim = np.array([0.0 for _ in n_drones])
# air_stdev_lim = np.array([0.0 for _ in n_drones])

for e in estimators:
    t_mean_inf[e] = np.array([0.0 for _ in n_drones])
    t_stdev_inf[e] = np.array([0.0 for _ in n_drones])
    t_q1_inf[e] = np.array([0.0 for _ in n_drones])
    t_median_inf[e] = np.array([0.0 for _ in n_drones])
    t_q3_inf[e] = np.array([0.0 for _ in n_drones])
    # t_mean_lim[e] = np.array([0.0 for _ in n_drones])
    # t_stdev_lim[e] = np.array([0.0 for _ in n_drones])



# comp time inf
fig, ax = plt.subplots(1,1)
for e in estimators:
    for i_n in range(len(n_drones)):
        t_mean_inf[e][i_n] = np.mean(all_comp_time_inf[e][i_n])/1000
        t_stdev_inf[e][i_n] = np.std(all_comp_time_inf[e][i_n])/1000
        t_q1_inf[e][i_n] = np.quantile(all_comp_time_inf[e][i_n], 0.25)/1000
        t_median_inf[e][i_n] = np.quantile(all_comp_time_inf[e][i_n], 0.5)/1000
        t_q3_inf[e][i_n] = np.quantile(all_comp_time_inf[e][i_n], 0.75)/1000
    # ax.plot(n_drones, t_mean_inf[e]/1000, label=names[e], 
    #         color=colors[e][1], linestyle='dashed', marker='x')
    # ax.errorbar(n_drones, t_median_inf[e], yerr=[t_mean_inf[e]-t_q1_inf[e], t_q3_inf[e]-t_mean_inf[e]], label=names[e], 
    #         color=colors[e][1], linestyle='dashed', marker='x')
    ax.plot(n_drones, t_mean_inf[e], label=names[e], 
            color=colors_est[e][1], linestyle='dashed', marker='x')
    # ax.fill_between(n_drones, t_q1_inf[e], t_q3_inf[e], facecolor=colors[e][0], alpha=0.4)
ax.legend()
ax.grid(True)
ax.set_xlabel(r'Total Number of Drones [-]')
ax.set_ylabel(r'Average Computation Time [ms]')
ax.set_xticks(n_drones)

if save_figures:
    save_name = 'comp_time_inf.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='pdf')


# air utilization
theoretical_n = np.linspace(n_drones[0],n_drones[-1])
theoretical_au = (theoretical_n * 20 * (1.1017 + 992/6.8))/1000000
fig, ax = plt.subplots(1,1)
for i_n in range(len(n_drones)):
    air_mean_inf[i_n] = np.mean(all_air_utilization_inf[i_n])
    air_stdev_inf[i_n] = np.std(all_air_utilization_inf[i_n])
    # air_mean_lim[i_n] = np.mean(all_air_utilization_lim[i_n])
    # air_stdev_lim[i_n] = np.std(all_air_utilization_lim[i_n])

ax.plot(n_drones, air_mean_inf, label='Observed', 
        color=colors_qualitative[0][1], linestyle='dashed', marker='x')
ax.text(3, 18.1, "ALOHA limit: 18\%", color='r')
ax.set_xlim(2, 43)
ax.set_ylim(0, 22)
ax.plot([*ax.get_xlim()], [18,18], color='r', linewidth=0.8)
# ax.errorbar(n_drones, air_mean_inf, yerr=air_stdev_inf, label='Infinite Range', 
#         color=color0[1], linestyle='dashed', marker='x')
# ax.plot(n_drones, air_mean_lim*100, label='Limited Range', 
#         color=color1[1], linestyle='dashed', marker='x')
# ax.fill_between(n_drones, mean_inf_range[e]-stdev_inf_range[e], mean_inf_range[e]+stdev_inf_range[e], facecolor=colors[e][0], alpha=0.4)
ax.grid(axis='y')
# ax.legend()
ax.set_xlabel(r'Total Number of Drones [-]')
ax.set_ylabel(r'Average Air Utilization [\%]')
ax.set_xticks(n_drones)


if save_figures:
    save_name = 'air_utilization.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='pdf')



plt.show()
