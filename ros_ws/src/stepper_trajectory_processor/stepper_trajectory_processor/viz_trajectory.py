import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from stepper_msgs.msg import StepperTrajectory, StepperTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

def save_joint_trajectory(trajectory: JointTrajectory):
    with open("joint_trajectory.csv", 'w') as fd:
        fd.write("sec,nanosec,")
        fd.write(",".join([name + "_position" for name in trajectory.joint_names])+",")
        fd.write(",".join([name + "_velocity" for name in trajectory.joint_names])+"\n")
        for point in trajectory.points:
            point: JointTrajectoryPoint
            fd.write(f"{point.time_from_start.sec},{point.time_from_start.nanosec},")
            fd.write(",".join([f"{p}" for p in point.positions])+",")
            fd.write(",".join([f"{v}" for v in point.velocities])+"\n")

def save_step_trajectory(trajectory: StepperTrajectory):
    with open("stepper_trajectory.csv", 'w') as fd:
        fd.write("microsec,")
        fd.write(",".join([name + "_steps" for name in trajectory.motor_names])+",")
        fd.write(",".join([name + "_steps_per_sec" for name in trajectory.motor_names])+"\n")
        for point in trajectory.points:
            point: StepperTrajectoryPoint
            fd.write(f"{point.time_from_start_us : .0f},")
            fd.write(",".join([f"{p}" for p in point.steps])+",")
            fd.write(",".join([f"{v}" for v in point.steps_per_sec])+"\n")

def encode(p1: StepperTrajectoryPoint, p2: StepperTrajectoryPoint, args):
    VMIN = 50
    dt = p2.time_from_start_us - p1.time_from_start_us
    retval = []
    for x2, v1, v2 in zip(p2.steps, p1.steps_per_sec, p2.steps_per_sec):
        # if there is no steps don't encode anything
        if x2 == 0:
            retval.append([])
            continue
        # if we are below minimum speed encode the average speed
        if abs(v1) < VMIN or abs(v2) < VMIN:
            retval.append([dt/x2]*abs(x2))
            continue
        # else we can do linear speed interpolation
        pulses = []
        t = p1.time_from_start_us
        for _ in range(abs(x2)):
            remainder = t - p1.time_from_start_us
            diff = v2 - v1
            v = v1 + diff*remainder/dt
            pulse = 1000000 / v
            pulses.append(int(pulse))
            t += abs(pulse)
        retval.append(pulses)
    return retval

def simulation_wrapper(trajectory: StepperTrajectory, wrap_fun, args=None):

    time = {name: 0 for name in trajectory.motor_names}
    value = {name: 0 for name in trajectory.motor_names}
    timeseries = {name: [(0,0)] for name in trajectory.motor_names}
    # interpolate through poitns
    p1 = None
    for p2 in trajectory.points:
        if not p1: 
            p1=p2
            continue
        p2: StepperTrajectoryPoint
        pulses = wrap_fun(p1, p2, args)
        for i, name in enumerate(trajectory.motor_names):
            for pulse in pulses[i]:
                time[name] += abs(pulse)*1e-6
                value[name] += 1 if pulse > 0 else -1
                timeseries[name].append((time[name],value[name]))
        p1=p2
    return timeseries

def plot_trajectory(trajectory: StepperTrajectory, ref: JointTrajectory, config):
    Xi = simulation_wrapper(trajectory, encode)
    fig, ax = plt.subplots(2,1)
    fig.set_size_inches(16, 9)

    # Use color cycling from a predefined color map
    colors = list(mcolors.TABLEAU_COLORS.values())
    color_cycle = iter(colors)

    for i, name in enumerate(trajectory.motor_names):
        # Get the next color in the cycle
        color = next(color_cycle)

        # Plot the actual trajectory
        ax[0].plot([x[0] for x in Xi[name]], [x[1] for x in Xi[name]],
                label=name,
                color=color)
        
        # Plot the reference trajectory with a dashed line
        Tref = [p.time_from_start_us/1e6 for p in trajectory.points]
        Nref = [p.steps[i] for p in trajectory.points]
        Xref = [sum(Nref[:j+1]) for j in range(len(Nref))]
        Vref = [p.steps_per_sec[i] for p in trajectory.points]
        ax[0].plot(Tref, Xref,
                label=f'ref {name}',
                color='k',
                linestyle='-.'
                )
        ax[1].plot(Tref, Vref,
                   label=f'ref {name}',
                   color=color,)
    for axis in ax:
        axis.grid()
        axis.legend()
    plt.show()

if __name__ == '__main__':
    
    traj = StepperTrajectory()
    traj.motor_names = ["m1", "m2"]
    dt = 0.2
    N = 40
    for i in range(N+1):
        p = StepperTrajectoryPoint()
        p.steps = [int(a*np.sin(i/N*np.pi)) for a in [100, -400]]
        p.steps_per_sec = [int(s/dt) for s in p.steps]
        p.time_from_start_us = int( i * dt * 1e6 )
        traj.points.append(p)
    for p in traj.points: print(p)
    plot_trajectory(traj, None, None)
    print("the end")