import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def plot_data(ax, data):
    allocation = data["allocation"]
    for p in allocation:
        ax.plot([data["drones"][p[0]][0], data["circles"][p[1]][0]], 
                [data["drones"][p[0]][1], data["circles"][p[1]][1]], 
                [data["drones"][p[0]][2], data["circles"][p[1]][2]])
    for d in data["drones"].values():
        ax.scatter(d[0], d[1], d[2], marker="o")
    for c in data["circles"].values():
        ax.scatter(c[0], c[1], c[2], marker="*", color="r")


flight_data_file = open("flight_data.json")
flight_data = json.load(flight_data_file)

ax1 = plt.axes(projection='3d')
plot_data(ax1, flight_data["5v5move"]["checkpoint1"])
ax2 = plt.axes(projection='3d')
plot_data(ax2, flight_data["5v5move"]["checkpoint2"])
plt.show()
