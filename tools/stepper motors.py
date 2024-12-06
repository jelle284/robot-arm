from matplotlib import pyplot as plt
import numpy as np

MIN_STEP_MS = 16
TABLE_POINTS = 100
# boundary conditions
N = 12000
t1 = 8.0


v0 = (t1/(MIN_STEP_MS*1e-3))/N
a = 2*v0-2;
b = 3-3*v0;
c = v0

print(a, b, c)

def dy(x):
    return 3*a*x**2 + 2*b*x + c

increment = 1.0/(TABLE_POINTS-1)
table = []
for i in range(TABLE_POINTS):
    x = increment*i
    f = dy(x)
    v = int(f * N/t1)
    table.append(int(500000 / v))

tb = int(1e6*t1/(TABLE_POINTS-1))
print("time base:", tb*1e6)
print("table data:")
for i in range(10):
    for j in range(10):
        value = table[i*10+j]
        print(f"{value},".ljust(8), end="")
    print("")
   
plotvar = {
    "tp": [],
    "m": [],
    "t": [],
    }

m = 0
t = 0

for i in range(N):
    # plan
    idx = int(t/tb)
    if idx >= TABLE_POINTS: idx = TABLE_POINTS - 1
    addit = 0
    if idx < TABLE_POINTS-1:
        rem = t - tb*idx;
        diff = table[idx+1]-table[idx]
        addit = diff*rem/tb
    tp = table[idx] + addit
    # log
    plotvar["tp"].append(tp)
    plotvar["m"].append(m)
    plotvar["t"].append(t)
    
    # step
    m+= 1
    t += 2*tp

print(m, "steps taken in ", t*1e-6, "seconds")
fig, ax = plt.subplots(2,1, constrained_layout = True)
ax[0].plot(plotvar["t"], [1/i for i in plotvar["tp"]])
ax[0].grid()
ax[1].plot(plotvar["t"], plotvar["m"])
ax[1].grid()

plt.show()