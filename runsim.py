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

def render(quad):
    frame = quad.world_frame()
    plt.set_frame(frame)

def attitudeControl(quad, time):
    desired_state = trajGen.genLine(time[0])
    F, M = controller.run(quad, desired_state)
    quad.update(dt, F, M)
    time[0] += dt
    #print quad.state[0], quad.state[1], quad.state[2]

def main():
    #pdb.set_trace()
    pos = (0,0,0)
    attitude = (0,0,np.pi/2)
    quadcopter = Quadcopter(pos, attitude)
    while time[0] < 1.0:
        attitudeControl(quadcopter,time)
        #print time[0]
    t = np.arange(0.,1.,dt)
    print t.shape
    print len(quadcopter.hist)
    plt.plot(quadcopter.hist)
    plt.show()


if __name__ == "__main__":
    main()
