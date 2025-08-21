import stepper_joint_conversion as cvt
import pandas as pd
from array import array

# Load motor configuration file
motor_conf = cvt.load_config("motors.yaml")
# create transformation matrices
tf, inv_tf = cvt.make_tf(motor_conf)
# read the trajectory file
df = pd.read_csv("trajectory.csv", index_col=0)

for i in range(len(df)):
    # Extract positions, velocities, and accelerations for the specified row
    pos = array('d', (df.loc[i, df.columns.str.contains('positions_')]).tolist())
    vel = array('d', (df.loc[i, df.columns.str.contains('velocities_')]).tolist())
    acc = array('d', df.loc[i, df.columns.str.contains('accelerations_')].tolist())

    spos = cvt.convert_joint_to_steps(pos, inv_tf)
    svel = cvt.convert_joint_to_steps(vel, inv_tf)
    sacc = cvt.convert_joint_to_steps(acc, inv_tf)
    print("next")
print("end")