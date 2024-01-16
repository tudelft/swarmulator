import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

import pandas as pd
import numpy as np

save_figures = False
dir_name = '2023-11-06-11:45:45_5_agents'

DATA_DIR = os.path.join(os.path.dirname(__file__), '../../../../logs', dir_name)

fig, ax = plt.subplots(4,1)

all_comp_time = { "ref" : np.array([]), 
                  "ful": np.array([]), 
                  "bnk": np.array([]), 
                  "dyn": np.array([]) }

for file in os.listdir(DATA_DIR):
    if file.startswith('.'):
        continue
    
    name, _ = file.split('.')
    _, _, id = file.split('_')
    data = pd.read_csv(os.path.join(DATA_DIR, file), skipinitialspace=True)

    if all_comp_time["ref"].size == 0:
        all_comp_time["ref"] = data["ref_t_us"].to_numpy()
        all_comp_time["ful"] = data["ful_t_us"].to_numpy()
        all_comp_time["bnk"] = data["bnk_t_us"].to_numpy()
        all_comp_time["dyn"] = data["dyn_t_us"].to_numpy()
    else:
        all_comp_time["ref"] = np.hstack((all_comp_time["ref"], data["ref_t_us"].to_numpy()))
        all_comp_time["ful"] = np.hstack((all_comp_time["ful"], data["ful_t_us"].to_numpy()))
        all_comp_time["bnk"] = np.hstack((all_comp_time["bnk"], data["bnk_t_us"].to_numpy()))
        all_comp_time["dyn"] = np.hstack((all_comp_time["dyn"], data["dyn_t_us"].to_numpy()))

    # all_comp_time["ref"].append(data["ref_t_us"].to_numpy())
    # all_comp_time["ful"].append(data["ful_t_us"].to_numpy())
    # all_comp_time["bnk"].append(data["bnk_t_us"].to_numpy())
    # all_comp_time["dyn"].append(data["dyn_t_us"].to_numpy())

    ax[0].plot(data["time"].to_numpy(), data["ref_c1_mean"].to_numpy(), linestyle='-', color='k', linewidth=0.6, label='C1')
    ax[0].plot(data["time"].to_numpy(), data["ref_c3_mean"].to_numpy(), linestyle='-', color='g', linewidth=0.6, label='C3')
    ax[0].plot(data["time"].to_numpy(), data["ref_c5_mean"].to_numpy(), linestyle='-', color='r', linewidth=0.6, label='C5')
    ax[0].plot(data["time"].to_numpy(), data["ref_icr_mean"].to_numpy(), linestyle='-', color='b', linewidth=0.6, label='ICR')
    

    ax[1].plot(data["time"].to_numpy(), data["ful_c1_mean"].to_numpy(), linestyle='-', color='k', linewidth=0.6, label='C1')
    ax[1].plot(data["time"].to_numpy(), data["ful_c3_mean"].to_numpy(), linestyle='-', color='g', linewidth=0.6, label='C3')
    ax[1].plot(data["time"].to_numpy(), data["ful_c5_mean"].to_numpy(), linestyle='-', color='r', linewidth=0.6, label='C5')
    ax[1].plot(data["time"].to_numpy(), data["ful_icr_mean"].to_numpy(), linestyle='-', color='b', linewidth=0.6, label='ICR')


    ax[2].plot(data["time"].to_numpy(), data["bnk_c1_mean"].to_numpy(), linestyle='-', color='k', linewidth=0.6, label='C1')
    ax[2].plot(data["time"].to_numpy(), data["bnk_c3_mean"].to_numpy(), linestyle='-', color='g', linewidth=0.6, label='C3')
    ax[2].plot(data["time"].to_numpy(), data["bnk_c5_mean"].to_numpy(), linestyle='-', color='r', linewidth=0.6, label='C5')
    ax[2].plot(data["time"].to_numpy(), data["bnk_icr_mean"].to_numpy(), linestyle='-', color='b', linewidth=0.6, label='ICR')


    ax[3].plot(data["time"].to_numpy(), data["dyn_c1_mean"].to_numpy(), linestyle='-', color='k', linewidth=0.6, label='C1')
    ax[3].plot(data["time"].to_numpy(), data["dyn_c3_mean"].to_numpy(), linestyle='-', color='g', linewidth=0.6, label='C3')
    ax[3].plot(data["time"].to_numpy(), data["dyn_c5_mean"].to_numpy(), linestyle='-', color='r', linewidth=0.6, label='C5')
    ax[3].plot(data["time"].to_numpy(), data["dyn_icr_mean"].to_numpy(), linestyle='-', color='b', linewidth=0.6, label='ICR')


plt_cmp_t, ax_cmp_t = plt.subplots()
ax_cmp_t.boxplot(all_comp_time["ref"], labels=["ref"], positions=[0])
ax_cmp_t.boxplot(all_comp_time["ful"], labels=["ful"], positions=[1])
ax_cmp_t.boxplot(all_comp_time["bnk"], labels=["bnk"], positions=[2])
ax_cmp_t.boxplot(all_comp_time["dyn"], labels=["dyn"], positions=[3])

# ax_cmp_t.boxplot(all_comp_time["ful"])
# ax_cmp_t.boxplot(all_comp_time["bnk"])
# ax_cmp_t.boxplot(all_comp_time["dyn"])

plt.show()
