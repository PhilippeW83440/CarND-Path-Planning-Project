import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
pathCollec1 = plt.scatter([10],[10],marker='o')
pathCollec1.set_figure(fig)
fig.show()

ax.cla()
pathCollec2 = plt.scatter(20,20,marker='x')
pathCollec2.set_figure(fig)
fig.show()
