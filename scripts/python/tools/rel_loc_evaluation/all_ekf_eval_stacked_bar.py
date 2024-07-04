import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.legend_handler import HandlerLine2D, HandlerTuple

import pandas as pd
import numpy as np
from plot_settings import *

save_figures = True

estimators = ['ref', 'dec', 'ful', 'dyn']
names = {'ref': '\n\n Centralized',
        'dec': '\n\n Decoupled',
        'ful': '\n\n Full State',
        'dyn': '\n\n Dynamic'}
errors = ['c1', 'c3', 'c5', 'icr']

start_time = 60 # let estimators converge

# Organize logs by type and number of drones
data_dir = os.path.join(os.path.dirname(__file__), 'data/all_ekf_evaluation')    
log_directories = [ f.path for f in os.scandir(data_dir) if f.is_dir() ]
log_type = ['N/A' for _ in log_directories]

# set up bins
error_count_bin = {'inf': dict(), 'lim': dict()}
error_count_total = {'inf': 0, 'lim': 0}

bin_categories = ['leq005', 'leq010', 'leq050', 'gt050', 'inv']
bin_labels = {
    'leq005': r'error $\leq$ 5\%',
    'leq010': r'error $\leq$ 10\%',
    'leq050': r'error $\leq$ 50\%',
    'gt050': r'error $>$ 50\%',
    'inv': r'No estimate',
}

for t in ['inf', 'lim']:
    for e in estimators:
        error_count_bin[t][e] = dict()
        for c in bin_categories:
            # errors: c1, c3, c5, icr
            error_count_bin[t][e][c] = np.array([0,0,0,0])
        
# Aggregate data from all drones
for i_log, log_path in enumerate(log_directories):
    _, name = os.path.split(log_path)
    if 'inf' in name:
        l_type = 'inf'
    elif 'lim' in name:
        l_type = 'lim'

    for file in os.listdir(log_path):
        if file.startswith('log_drone_') and ("relpos" not in file):
            name, _ = file.split('.')
            data = pd.read_csv(os.path.join(log_path, file), skipinitialspace=True)

            start_index = 0
            while(data["time"].iloc[start_index]<start_time):
                start_index += 1
            data = data.iloc[start_index:,:]
            error_count_total[l_type] += len(data["time"])

            for e in estimators:             
                for i_err, err in enumerate(errors):
                    field = e + "_" + err + "_rel"
                    
                    error_count_bin[l_type][e]['inv'][i_err] += len(data[data[field]<0])
                    error_count_bin[l_type][e]['leq005'][i_err] += len(data[(data[field] >= 0) & (data[field]<=0.05)])
                    error_count_bin[l_type][e]['leq010'][i_err] += len(data[(data[field] > 0.05) & (data[field]<=0.1)])
                    error_count_bin[l_type][e]['leq050'][i_err] += len(data[(data[field] > 0.1) & (data[field]<=0.5)])
                    error_count_bin[l_type][e]['gt050'][i_err] += len(data[data[field]>0.5])


error_frac_bin = {'inf': dict(), 'lim': dict()}
for c in bin_categories:
    error_frac_bin['inf'][c] = []
    error_frac_bin['lim'][c] = []
    for i_est, e in enumerate(estimators):
        for i_cat in range(len(errors)):
            error_frac_bin['inf'][c].append(100*error_count_bin['inf'][e][c][i_cat]/error_count_total['inf'])
            error_frac_bin['lim'][c].append(100*error_count_bin['lim'][e][c][i_cat]/error_count_total['lim'])

#####################################################
# Plot 1 - relative errors bar sequential color Inf #
#####################################################
fig, ax = plt.subplots(1,1)

pos = [0,1,2,3, 5,6,7,8, 10,11,12,13, 15,16,17,18]
labels_all = [*errors, *errors, *errors, *errors]
bottom = np.zeros(16)
i = 0
for name, frac in error_frac_bin['inf'].items():
    p = ax.bar(pos, frac, color=colors_seq5[i][0], label=bin_labels[name], bottom=bottom)
    bottom += frac
    i+=1
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

ax.set_xticks(pos)
ax.set_xticklabels([*errors,*errors,*errors,*errors])

ax.set_xticks([1.5, 6.5, 11.5, 16.5], minor=True)
ax.set_xticklabels([names['ref'], names['dec'], names['ful'], names['dyn']], minor=True)
ax.tick_params(which='minor', length=0)
plt.gca().set_aspect(0.1)

# legend_elements = []
# for i, e in enumerate(estimators):
#     legend_elements.append(Patch(facecolor=colors_qualitative[i][0],
#                                  edgecolor=colors_qualitative[i][1],
#                                  label=names[e]))

# ax.legend(handles=legend_elements, ncol=1, loc="upper right")

ax.set_axisbelow(True)
ax.grid(True, axis='y')
# ax.legend(loc="lower right")
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1], loc='lower right')

# ax.set_ylim(0,0.5)
ax.set_ylabel('Fraction of relative errors [-]')
if save_figures:
    save_name = 'all_ekf_rel_inf_bar.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    # plt.savefig(fname, format='pdf')
    plt.savefig(fname, format='pdf', bbox_inches='tight')


######################################################
# Plot 2 - relative errors bar qualitative color Inf #
######################################################
fig, ax = plt.subplots(1,1)
p = dict()
for i,e in enumerate(estimators):
    pos = [5*i, 5*i+1, 5*i+2, 5*i+3]
    bottom = np.zeros(4)
    i_err = 0
    p[e] = dict()
    for name, count in error_count_bin['inf'][e].items():
        frac = 100*count/error_count_total['inf']
        p[e][name] = ax.bar(pos, frac, color=colors_quali_5shades[i][4-i_err], label=bin_labels[name], bottom=bottom)
        bottom += frac
        i_err += 1


ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

pos = [0,1,2,3, 5,6,7,8, 10,11,12,13, 15,16,17,18]
ax.set_xticks(pos)
ax.set_xticklabels([*errors,*errors,*errors,*errors])

ax.set_xticks([1.5, 6.5, 11.5, 16.5], minor=True)
ax.set_xticklabels([names['ref'], names['dec'], names['ful'], names['dyn']], minor=True)
ax.tick_params(which='minor', length=0)
plt.gca().set_aspect(0.1)

# legend_elements = []
# for i, e in enumerate(estimators):
#     legend_elements.append(Patch(facecolor=colors_qualitative[i][0],
#                                  edgecolor=colors_qualitative[i][1],
#                                  label=names[e]))

# ax.legend(handles=legend_elements, ncol=1, loc="upper right")


ax.set_axisbelow(True)
ax.grid(True, axis='y')
# ax.legend(loc="lower right")

tpls = []
labels = []
for key in bin_categories:
    tpls.append((p[estimators[0]][key], p[estimators[1]][key], p[estimators[2]][key], p[estimators[3]][key]))
    labels.append(bin_labels[key])
ax.legend(tpls[::-1], labels[::-1], handler_map={tuple:HandlerTuple(ndivide=None, pad=0.1)}, handlelength=3, loc='lower right')
# handles, labels = ax.get_legend_handles_labels()
# ax.legend(handles[::-1], labels[::-1], loc='lower right')

# ax.set_ylim(0,0.5)
ax.set_ylabel('Fraction of relative errors [-]')
if save_figures:
    save_name = 'all_ekf_rel_inf_bar_cat_color.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    # plt.savefig(fname, format='pdf')
    plt.savefig(fname, format='pdf', bbox_inches='tight')

#####################################################
# Plot 3 - relative errors bar sequential color Lim #
#####################################################
fig, ax = plt.subplots(1,1)

pos = [0,1,2,3, 5,6,7,8, 10,11,12,13, 15,16,17,18]
labels_all = [*errors, *errors, *errors, *errors]
bottom = np.zeros(16)
i = 0
for name, frac in error_frac_bin['lim'].items():
    p = ax.bar(pos, frac, color=colors_seq5[i][0], label=bin_labels[name], bottom=bottom)
    bottom += frac
    i+=1
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

ax.set_xticks(pos)
ax.set_xticklabels([*errors,*errors,*errors,*errors])

ax.set_xticks([1.5, 6.5, 11.5, 16.5], minor=True)
ax.set_xticklabels([names['ref'], names['dec'], names['ful'], names['dyn']], minor=True)
ax.tick_params(which='minor', length=0)
plt.gca().set_aspect(0.1)

# legend_elements = []
# for i, e in enumerate(estimators):
#     legend_elements.append(Patch(facecolor=colors_qualitative[i][0],
#                                  edgecolor=colors_qualitative[i][1],
#                                  label=names[e]))

# ax.legend(handles=legend_elements, ncol=1, loc="upper right")


ax.set_axisbelow(True)
ax.grid(True, axis='y')
# ax.legend(loc="lower right")
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1], loc='lower right')

# ax.set_ylim(0,0.5)
ax.set_ylabel('Fraction of relative errors [-]')
if save_figures:
    save_name = 'all_ekf_rel_lim_bar.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    # plt.savefig(fname, format='pdf')
    plt.savefig(fname, format='pdf', bbox_inches='tight')


######################################################
# Plot 4 - relative errors bar qualitative color Lim #
######################################################
fig, ax = plt.subplots(1,1)

p = dict()
for i,e in enumerate(estimators):
    pos = [5*i, 5*i+1, 5*i+2, 5*i+3]
    bottom = np.zeros(4)
    i_err = 0
    p[e] = dict()
    for name, count in error_count_bin['lim'][e].items():
        frac = 100*count/error_count_total['lim']
        p[e][name] = ax.bar(pos, frac, color=colors_quali_5shades[i][4-i_err], label=bin_labels[name], bottom=bottom)
        bottom += frac
        i_err += 1


ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

pos = [0,1,2,3, 5,6,7,8, 10,11,12,13, 15,16,17,18]
ax.set_xticks(pos)
ax.set_xticklabels([*errors,*errors,*errors,*errors])

ax.set_xticks([1.5, 6.5, 11.5, 16.5], minor=True)
ax.set_xticklabels([names['ref'], names['dec'], names['ful'], names['dyn']], minor=True)
ax.tick_params(which='minor', length=0)
plt.gca().set_aspect(0.1)

# legend_elements = []
# for i, e in enumerate(estimators):
#     legend_elements.append(Patch(facecolor=colors_qualitative[i][0],
#                                  edgecolor=colors_qualitative[i][1],
#                                  label=names[e]))

# ax.legend(handles=legend_elements, ncol=1, loc="upper right")


ax.set_axisbelow(True)
ax.grid(True, axis='y')
# ax.legend(loc="lower right")

tpls = []
labels = []
for key in bin_categories:
    tpls.append((p[estimators[0]][key], p[estimators[1]][key], p[estimators[2]][key], p[estimators[3]][key]))
    labels.append(bin_labels[key])
ax.legend(tpls[::-1], labels[::-1], handler_map={tuple:HandlerTuple(ndivide=None, pad=0.1)}, handlelength=3, loc='lower right')
# handles, labels = ax.get_legend_handles_labels()
# ax.legend(handles[::-1], labels[::-1], loc='lower right')

# ax.set_ylim(0,0.5)
ax.set_ylabel('Fraction of relative errors [-]')
if save_figures:
    save_name = 'all_ekf_rel_lim_bar_cat_color.pdf'
    fname = os.path.join(DIR_PLOTS, save_name)
    # plt.savefig(fname, format='pdf')
    plt.savefig(fname, format='pdf', bbox_inches='tight')


plt.show()