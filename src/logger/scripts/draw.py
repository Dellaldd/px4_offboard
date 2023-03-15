from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
file_name = "test.txt"
save_name = "test.png"
read_path = "/home/ldd/UAV/rosbag/" + file_name
data = np.loadtxt(read_path, delimiter=',')
fig, ax = plt.subplots(4, 3)
for i in range(3):
    ax[0][i].plot(data[:,0], data[:,i+1], 'r*', label="local")

ax[0][0].plot(data[:,0], 1*np.ones((data.shape[0],1)), 'b-', label = "target")
ax[0][1].plot(data[:,0], 1*np.ones((data.shape[0],1)), 'b-', label = "target")
ax[0][2].plot(data[:,0], 1.5*np.ones((data.shape[0],1)), 'b-', label = "target")

for i in range(3):
    for j in range(3):
        ax[i+1][j].plot(data[:,0], data[:,i*6+j+7], 'r*', label = "local")  # 7  8  9     13 14 15   19 20 21
        ax[i+1][j].plot(data[:,0], data[:,i*6+j+10], 'b-', label = "target") # 10 11 12    16 17 18   22 23 24

ax[0, 0].set_title("position x")
ax[0, 1].set_title("position y")
ax[0, 2].set_title("position z")
ax[1, 0].set_title("velocity x")
ax[1, 1].set_title("velocity y")
ax[1, 2].set_title("velocity z")
ax[2, 0].set_title("attitude x")
ax[2, 1].set_title("attitude y")
ax[2, 2].set_title("attitude z")
ax[3, 0].set_title("body_rate x")
ax[3, 1].set_title("body_rate y")
ax[3, 2].set_title("body_rate z")
lines, labels = fig.axes[-1].get_legend_handles_labels()
fig.legend(lines, labels)
fig.tight_layout()
save_path = "/home/ldd/UAV/rosbag/" + save_name
plt.savefig(save_path, dpi=300)
plt.show()





