import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import math
x = np.linspace(0,6,200)
y = []
z = []
print(x)
for i in x:
    y.append(math.sin(60*i/180*math.pi))
    z.append(1-math.cos(60*i/180*math.pi))
plt.figure()
plt.plot(y,z,label="$sin(x)$",color="red",linewidth=2)

plt.show()
