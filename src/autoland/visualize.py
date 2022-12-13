from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

def plot_trajectory(states_ned):
    '''
        Trajectory in NED coordinates
        states_ned is 3xN
    '''

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.invert_zaxis()

    ax.plot3D(states_ned[0,:],
              states_ned[1,:],
              states_ned[2,:],
              'gray')
    plt.show(block=False)