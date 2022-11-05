import numpy as np
import matplotlib.pyplot as plt

a = np.loadtxt('mag_cal.txt', delimiter=',', skiprows=1)
mag = []

for x, y, z in a:
    if x > 80 or y > 80 or z > 210:
        continue
    mag.append([x, y, z])
mag = np.array(mag)

mx = mag[:, 0]
my = mag[:, 1]
mz = mag[:, 2]

xmax, xmin = np.max(mx), np.min(mx)
ymax, ymin = np.max(my), np.min(my)
zmax, zmin = np.max(mz), np.min(mz)

print(xmax, xmin, ymax, ymin, zmax, zmin)

mx -= (xmax + xmin) / 2
my -= (ymax + ymin) / 2
mz -= (zmax + zmin) / 2

dx = (xmax - xmin) / 2
dy = (ymax - ymin) / 2
dz = (zmax - zmin) / 2

davg = (dx + dy + dz) / 3

mx *= davg / dx
my *= davg / dy
mz *= davg / dz

fig = plt.figure()
ax = fig.subplots(1, 1)

ax.scatter(mx, my)
ax.scatter(mx, mz)
ax.scatter(my, mz)
ax.set_aspect('equal', 'box')

plt.show()
