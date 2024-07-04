import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.legend_handler import HandlerLine2D, HandlerTuple

import pandas as pd
import numpy as np
from plot_settings import *

save_figures = True

est_type = 'ful'
# est_type = 'dyn'

estimators = ['noo', 'a00', 'b00', 'ab0', 'abc', 'abd', 'abe', 'all']
# names = {'noo': "No improvements",
#          'c00': "Initialization",
#          'd00': "Selective Secondary",
#          'e00': "Covariance Inflation",
#          'f00': "Withhold Secondary",
#          'g00': "Reset Agent",
#          'all': "All improvements"}
names = {'noo': "None",
         'a00': "A",
         'b00': "B",
         'ab0': "A+B",
         'abc': "A+B\n+C",
         'abd': "A+B\n+D",
         'abe': "A+B\n+E",
         'all': "All"}
errors = ['c1', 'c3', 'c5', 'icr']

start_time = 60 # let estimators converge

# Organize logs by type and number of drones
dir_name = 'data/ablation_study_' + est_type
data_dir = os.path.join(os.path.dirname(__file__), dir_name)    
log_directories = [ f.path for f in os.scandir(data_dir) if f.is_dir() ]
log_type = ['N/A' for _ in log_directories]

# for i_log, log_path in enumerate(log_directories):
#     _, name = os.path.split(log_path)
#     if 'inf' in name:
#         log_type[i_log] = 'inf'
#     elif 'lim' in name:
#         log_type[i_log] = 'lim'


#start_index = int(start_time/0.01)

# set up bins
error_count_bin = dict()
nis_count_bin = dict()
count_total = 0

error_bin_categories = ['leq005', 'leq010', 'leq050', 'gt050', 'inv']
error_bin_labels = {
    'leq005': r'error $\leq$ 5\%',
    'leq010': r'error $\leq$ 10\%',
    'leq050': r'error $\leq$ 50\%',
    'gt050': r'error $>$ 50\%',
    'inv': r'No estimate',
}

# with 9 drones tracked, nis should follow chi^2 distribution with
# 9*20 dof (averaged over 20 past measurements)
nis_bin_categories = ['q90','q95', 'q99', 'q999', 'qall']
gaussian_tails = [1.28, 1.64, 2.33, 3.09]
# nis_thresholds = [(0.5*(G+np.sqrt(2*180-1))**2) for G in gaussian_tails]
nis_bin_labels = {
    'q90': "$Q=0.9$",
    'q95': "$Q=0.95$",
    'q99': "$Q=0.99$", 
    'q999': "$Q=0.999$",
    'qall': "All"
}

for e in estimators:
    error_count_bin[e] = dict()
    nis_count_bin[e] = dict()
    for c in error_bin_categories:
        error_count_bin[e][c] = np.array([0,0,0,0])
        # errors: c1, c3, c5, icr
    for c in nis_bin_categories:
        nis_count_bin[e][c] = 0
        
# Aggregate data from all drones
for i_log, log_path in enumerate(log_directories):
    _, name = os.path.split(log_path)

    for file in os.listdir(log_path):
        if file.startswith('log_drone_'):
            if "relpos" in file:
                continue
            name, _ = file.split('.')
            data = pd.read_csv(os.path.join(log_path, file), skipinitialspace=True)

            start_index = 0
            while(data["time"].iloc[start_index]<start_time):
                start_index += 1
            data = data.iloc[start_index:,:]
            count_total += len(data["time"])

            for e in estimators: 
                dof = data[e+'_nis_dof']
                nis_thresholds = [(0.5*(G+np.sqrt(2*dof-1))**2) for G in gaussian_tails]
                field = e + "_nis_sum"
                nis_count_bin[e]['q90'] += len(data[data[field]<nis_thresholds[0]])
                nis_count_bin[e]['q95'] += len(data[(data[field] >= nis_thresholds[0]) & (data[field]<=nis_thresholds[1])])
                nis_count_bin[e]['q99'] += len(data[(data[field] > nis_thresholds[1]) & (data[field]<=nis_thresholds[2])])
                nis_count_bin[e]['q999'] += len(data[(data[field] > nis_thresholds[2]) & (data[field]<=nis_thresholds[3])])
                nis_count_bin[e]['qall'] += len(data[data[field]>nis_thresholds[3]])
               
                for i_err, err in enumerate(errors):
                    field = e + "_" + err + "_rel"
                    
                    error_count_bin[e]['inv'][i_err] += len(data[data[field]<0])
                    error_count_bin[e]['leq005'][i_err] += len(data[(data[field] >= 0) & (data[field]<=0.05)])
                    error_count_bin[e]['leq010'][i_err] += len(data[(data[field] > 0.05) & (data[field]<=0.1)])
                    error_count_bin[e]['leq050'][i_err] += len(data[(data[field] > 0.1) & (data[field]<=0.5)])
                    error_count_bin[e]['gt050'][i_err] += len(data[data[field]>0.5])


#################################################
# Plot 1 - relative errors bar sequential color #
#################################################
fig, ax = plt.subplots(1,1)

error_frac_bin = dict()
for c in error_bin_categories:
    error_frac_bin[c] = []
    for i_est, e in enumerate(estimators):
        # only use icr
        error_frac_bin[c].append(100*error_count_bin[e][c][3]/count_total)

        # for i_cat in range(len(errors)):
        #     error_frac_bin[c].append(100*error_count_bin[e][c][i_cat]/count_total)



pos = [0,2,3,5,6,7,8,10]
labels_all = [*errors, *errors, *errors, *errors]
bottom = np.zeros(len(pos))
i = 0
for name, frac in error_frac_bin.items():
    p = ax.bar(pos, frac, color=colors_seq5[i][0], label=error_bin_labels[name], bottom=bottom)
    bottom += frac
    i+=1
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

xlabels = []
for e in estimators:
    xlabels.append(names[e])
ax.set_xticks(pos)
ax.set_xticklabels([*xlabels])#, ha='left', rotation=-45, rotation_mode='anchor')
# ax.tick_params(axis='x', labelrotation=-45, rotation_mode='anchor')

# ax.set_xticks([1.5, 6.5, 11.5, 16.5], minor=True)
# ax.set_xticklabels([names['05d'], names['08d'], names['10d'], names['15d']], minor=True)
# ax.tick_params(which='minor', length=0)
plt.gca().set_aspect(0.08)

ax.set_axisbelow(True)
# ax.yaxis.grid(color='black')
ax.grid(True, axis='y')
# ax.legend(loc="lower right")
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1], loc='lower right')

# ax.set_ylim(0,0.5)
ax.set_ylabel('Fraction of relative errors [-]')
if save_figures:
    save_name = 'rel_error_ablation_bar_' + est_type + '.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    # plt.savefig(fname, format='pdf')
    plt.savefig(fname, format='pdf', bbox_inches='tight')

plt.show()