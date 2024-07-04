import os
import matplotlib.pyplot as plt
# from matplotlib.patches import Patch
from matplotlib.lines import Line2D
import pandas as pd
import numpy as np
from plot_settings import *

save_figures = True
data_dir = os.path.join(os.path.dirname(__file__), 'data/all_ekf_evaluation/dyn_ekf_evaluation_inf5')    
# data_dir = os.path.join(os.path.dirname(__file__), 'data/test')    
drone_no = 0

estimators = ['ref', 'dec', 'ful', 'dyn']
names = {'ref': 'Centralized',
        'dec': 'Decoupled',
        'ful': 'Full State',
        'dyn': 'Dynamic'}

# Time Series
file_name = 'log_drone_'+ str(drone_no) +'.csv'
data = pd.read_csv(os.path.join(data_dir,file_name), skipinitialspace=True)
for k in data.keys():
    data.loc[data[k] < 0, k] = np.nan

time_np = data["time"].to_numpy()

fig, ax = plt.subplots(4,1, sharex=True)

for i,e in enumerate(estimators):
    c1_np = data[e+"_c1_rel"].to_numpy()
    c3_np = data[e+"_c3_rel"].to_numpy()
    c5_np = data[e+"_c5_rel"].to_numpy()
    icr_np = data[e+"_icr_rel"].to_numpy()

    ax[i].plot(time_np, c1_np, linestyle='-', color='k', linewidth=0.6, label='C1')
    ax[i].plot(time_np, c3_np, linestyle='-', color='g', linewidth=0.6, label='C3')
    ax[i].plot(time_np, c5_np, linestyle='-', color='r', linewidth=0.6, label='C5')
    ax[i].plot(time_np, icr_np, linestyle='-', color='b', linewidth=0.6, label='ICR')
    # ax[i].set_title(names[e])
    ax[i].set_ylim([0,1])
    ax[i].set_ylabel(names[e])
    # ax[i].grid(True, axis='x')
    ax[i].fill_between(time_np, 0, 1, where=np.isnan(c1_np), facecolor='k', alpha=.2)
    ax[i].fill_between(time_np, 0, 1, where=(np.logical_and(np.isnan(c3_np), ~np.isnan(c1_np))), facecolor='g', alpha=.2)
    ax[i].fill_between(time_np, 0, 1, where=(np.logical_and(np.isnan(c5_np), ~np.isnan(c3_np))), facecolor='r', alpha=.2)

ax[0].legend(loc='upper right', ncol=4)
ax[3].set_xlabel("Simulation time [s]")
# plt.subplots_adjust(hspace=0.5)
if save_figures:
    save_name = 'time_serie.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='pdf')



# Rel pos
rp_start_time = 150
rp_end_time = 180

file_name = 'log_drone_'+ str(drone_no) +'_relpos.csv'
data = pd.read_csv(os.path.join(data_dir,file_name), skipinitialspace=True)

start_index = int(rp_start_time/0.01)
end_index = int(rp_end_time/0.01)
# while(data["time"].iloc[start_index]<rp_start_time):
#     start_index += 1
#     end_index += 1
# while(data["time"].iloc[end_index]<rp_end_time):
#     end_index += 1
data = data.iloc[start_index:end_index,:]

time_np = data["time"].to_numpy()

fig, ax = plt.subplots(1,1)

limits = 25
ax_limits = limits
ax.set_xlim([-ax_limits, ax_limits])
ax.set_ylim([-ax_limits, ax_limits])
l_self=ax.plot(0,0, marker='^', color = 'r', markersize=10, linewidth=0, label='self')
ax.text(0.5,0.5,str(drone_no), color='r')

for i_agent in range(1,10):
    fx = 'x'+str(i_agent)
    fy = 'y'+str(i_agent)

    dx = data[fx].to_numpy()
    dy = data[fy].to_numpy()
    ax.plot(dx, dy, linestyle='-', color='#AAAAAA', linewidth=0.6)
    ax.plot(dx[-1], dy[-1], marker='^', color='#AAAAAA')
    ax.text(dx[-1]+0.2, dy[-1]+0.2, str(i_agent))

    if (dx[-1]>-limits and dx[-1]<limits 
        and dy[-1]>-limits and dy[-1]<limits):
        for i,e in enumerate(estimators):
            dx = data[e+"_"+fx].to_numpy()
            dy = data[e+"_"+fy].to_numpy()
            ax.plot(dx, dy, linestyle='-', color=colors_est[e][1], linewidth=0.6)
            ax.plot(dx[-1], dy[-1], marker='^', color=colors_est[e][1])

legend_elements_lines = [Line2D([0],[0], color='r', marker='^', markersize=10, linewidth=0),
                         Line2D([0],[0], linestyle='-', marker='^', color='#AAAAAA', linewidth=1),
                         Line2D([0],[0], linestyle='-', marker='^', color=colors_est['ref'][1], linewidth=1),
                         Line2D([0],[0], linestyle='-', marker='^', color=colors_est['dec'][1], linewidth=1),
                         Line2D([0],[0], linestyle='-', marker='^', color=colors_est['ful'][1], linewidth=1),
                         Line2D([0],[0], linestyle='-', marker='^', color=colors_est['dyn'][1], linewidth=1),
                         ]

legend_elements_names = ['self', 'ground truth', names['ref'], names['dec'], names['ful'], names['dyn']]
ax.legend(legend_elements_lines, legend_elements_names)
ax.grid(True)

if save_figures:
    save_name = '2D_relpos.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='pdf')


#     ax[i].plot(time_np, c1_np, linestyle='-', color='k', linewidth=0.6, label='C1')
#     ax[i].plot(time_np, c3_np, linestyle='-', color='g', linewidth=0.6, label='C3')
#     ax[i].plot(time_np, c5_np, linestyle='-', color='r', linewidth=0.6, label='C5')
#     ax[i].plot(time_np, icr_np, linestyle='-', color='b', linewidth=0.6, label='ICR')
#     # ax[i].set_title(names[e])
#     ax[i].set_ylim([0,1])
#     ax[i].set_ylabel(names[e])
#     # ax[i].grid(True, axis='x')
#     ax[i].fill_between(time_np, 0, 1, where=np.isnan(c1_np), facecolor='k', alpha=.2)
#     ax[i].fill_between(time_np, 0, 1, where=(np.logical_and(np.isnan(c3_np), ~np.isnan(c1_np))), facecolor='g', alpha=.2)
#     ax[i].fill_between(time_np, 0, 1, where=(np.logical_and(np.isnan(c5_np), ~np.isnan(c3_np))), facecolor='r', alpha=.2)

# ax[0].legend(loc='upper right', ncol=4)
# ax[3].set_xlabel("Simulation time [s]")
# # plt.subplots_adjust(hspace=0.5)
# if save_figures:
#     save_name = 'time_serie.pdf'
#     fname = os.path.join(DIR_PLOTS, save_name)
#     plt.savefig(fname, format='pdf')


plt.show()
