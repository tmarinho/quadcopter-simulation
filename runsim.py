"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD

Modified by Thiago Marinho
email: marinho@illinois.edu
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import pdb
import matplotlib.pyplot as plt
import controller
import trajGen
from model.quadcopter import Quadcopter
import numpy as np

control_frequency = 200 # Hz for attitude control loop
dt = 1.0 / control_frequency
time = [0.0]
SIMTIME = 5.0
t = np.arange(0.,SIMTIME+dt,dt)
reward = -9999

def reward_fcn(states, des_states, F, M):
    x = states - des_states
    reward = - np.dot(x,x)
    #r = np.dot(M.T,M)[0,0]


def attitudeControl(quad, time):
    desired_state = trajGen.genLine(time[0])
    F, M = controller.run(quad, desired_state)
    reward_fcn(np.concatenate((quad.position(), quad.velocity()),axis=0),np.concatenate((desired_state.pos, desired_state.vel),axis=0),F,M)
    quad.update(dt, F, M)
    time[0] += dt

def main():
    #pdb.set_trace()
    pos = (0,0,0)
    attitude = (0,0,np.pi/2)
    quadcopter = Quadcopter(pos, attitude)

    # Simulation Loop
    while time[0] <= SIMTIME:
        attitudeControl(quadcopter,time)

    plt.plot(t,quadcopter.hist)
    plt.show()


if __name__ == "__main__":
    main()
