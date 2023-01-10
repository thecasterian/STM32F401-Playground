import numpy as np

a = np.loadtxt('gyro_cal.txt', delimiter=',', skiprows=1)

print(np.mean(a, axis=0))
