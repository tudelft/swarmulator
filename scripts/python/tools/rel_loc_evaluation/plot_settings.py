import os
import matplotlib.pyplot as plt
from pathlib import Path

DIR_PLOTS = os.path.join(os.path.dirname(__file__), 'plots')
Path(DIR_PLOTS).mkdir(exist_ok=True)

# Set parameters for plot fonts
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
})

boxplot_kwargs = {
    'widths': 0.5,
    'medianprops': dict(color='black')
}

# [face_color, line_color]
colors_qualitative = [
    ['#2b83ba', '#075181'],  # blue
    ['#abdda4', '#50a044'],  # green
    ['#ffaa7b', '#d98f68'],  # orange
    ['#c66bff', '#9952c4']  # purple
]

colors_div5 = [
    ['#3d7fb6','#285376'],
    ['#9eb6d3','#8495a8'],
    ['#f1f1f1','#dddddd'],
    ['#e4a6b1','#b98c94'],
    ['#cc5875','#8f3d52']  
]

colors_seq5 = [
    ['#00304a', '#002133'],
    ['#375b74', '#1f3e54'],
    ['#6689a0', '#3d5d77'],
    ['#98bbce', '#5c7f9b'],
    ['#cdefff', '#7ca2c2'],
]

blue_5shades = ['#87aed3','#527faa', '#346290', '#1d4974', '#092c4d']
red_5shades = ['#ffa79b','#ff8170', '#dd5644', '#b33322', '#771305']
green_5shades = ['#88e09f','#56c272', '#33a551', '#1a8636', '#04591a']
yellow_5shades = ['#ffd69b','#ffc570', '#dd9f44', '#b37822', '#774805']

# 5 shades light to dark
colors_quali_5shades =[
    red_5shades,
    blue_5shades,
    yellow_5shades,
    green_5shades
]

# color0 = ['#2b83ba', '#075181']  # blue
# color1 = ['#abdda4', '#50a044']  # green
# color2 = ['#ffaa7b', '#d98f68']  # orange
# color3 = ['#c66bff', '#9952c4']  # purple

colors_est = {'ref': colors_qualitative[0],
              'dyn': colors_qualitative[1],
              'ful': colors_qualitative[2],
              'dec': colors_qualitative[3]}


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