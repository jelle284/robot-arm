# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 22:05:47 2024

@author: Jelle
"""

import ikpy.chain
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
#%%
my_chain = ikpy.chain.Chain.from_urdf_file("resources/6dof-arm.urdf")
angles= [np.deg2rad(i) for i in [
        0,
        20.00,
        30.00,
        15.00,
        60.00,
        0.00,
        0.00,
        ]]

fk = my_chain.forward_kinematics(angles)
target = np.array([-0.4, 0.05, 0.1 ])

final = my_chain.inverse_kinematics(target)
#%%

ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

my_chain.plot(final, ax)
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
matplotlib.pyplot.show()
