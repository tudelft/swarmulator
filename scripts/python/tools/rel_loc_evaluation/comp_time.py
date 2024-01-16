import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

import pandas as pd
import numpy as np

save_figures = False

estimators = ['ref', 'ful', 'bnk', 'dyn']
face_colors = {
    'grey': '#DEDEDE',
    'green': '#2C8437'
}
line_colors = {
    'grey': '#B0B0B0',
    'green': '#1D5624'
}
directories = {
    "5ag" : '2023-11-07-10:14:53_5_agents',
    "10ag": '2023-11-07-10:15:29_10_agents',
    "20ag": '2023-11-07-10:17:00_20_agents'
} 

def boxplot_kwargs_with_colors(face_color, line_color):
    bp_kwargs = {
        'widths': 0.5,
        'patch_artist': True,
        'boxprops': dict(color=line_color, facecolor=face_color),
        'medianprops': dict(color=line_color),
        'capprops': dict(color=line_color),
        'flierprops': dict(markeredgecolor=line_color),
        'whiskerprops': dict(color=line_color),
        'showfliers': False
    }
    return bp_kwargs


all_comp_time = {}
plt_cmp_t, ax_cmp_t = plt.subplots()
plt.yscale('log')
plt_counter = 0
for nag, dir in directories.items():
    data_dir = os.path.join(os.path.dirname(__file__), '../../../../logs', dir)

    all_comp_time[nag] = {}
    for e in estimators:
        all_comp_time[nag][e] = np.array([0])


    for file in os.listdir(data_dir):
        if file.startswith('.'):
            continue
        
        name, _ = file.split('.')
        _, _, id = file.split('_')
        data = pd.read_csv(os.path.join(data_dir, file), skipinitialspace=True)

        for e in estimators:
            key = e + "_t_us"
            all_comp_time[nag][e] = np.hstack((all_comp_time[nag][e], data[key].to_numpy()))


    for i,e in enumerate(estimators):
        pos = len(estimators) * plt_counter + plt_counter + i
        if e == 'dyn':
            bp_kwargs = boxplot_kwargs_with_colors(face_colors['green'], line_colors['green'])
        else:
            bp_kwargs = boxplot_kwargs_with_colors(face_colors['grey'], line_colors['grey'])

        ax_cmp_t.boxplot(all_comp_time[nag][e], labels=[e], positions=[pos], **bp_kwargs)
    
    plt_counter += 1
    # ax_cmp_t.boxplot(all_comp_time["ref"], labels=["ref"], positions=[0])
    # ax_cmp_t.boxplot(all_comp_time["ful"], labels=["ful"], positions=[1])
    # ax_cmp_t.boxplot(all_comp_time["bnk"], labels=["bnk"], positions=[2])
    # ax_cmp_t.boxplot(all_comp_time["dyn"], labels=["dyn"], positions=[3])

# ax_cmp_t.boxplot(all_comp_time["ful"])
# ax_cmp_t.boxplot(all_comp_time["bnk"])
# ax_cmp_t.boxplot(all_comp_time["dyn"])

plt.show()
