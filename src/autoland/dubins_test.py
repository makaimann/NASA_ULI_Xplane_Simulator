from dynamics import DubinsAirplane
import math
import numpy as np
from visualize import plot_trajectory

if __name__ == '__main__':
    NUM_STEPS = 200
    da = DubinsAirplane()
    states = np.zeros((4, NUM_STEPS))

    def controller(state, i):
        V     = 1.0
        gamma = 0.5*math.sin(2 * math.pi / 8 * 0.05 * i)
        phi   = math.sin(2 * math.pi / 4 * 0.05 * i)
        return V, gamma, phi

    states[:, 0] = da.state
    for i in range(1, NUM_STEPS):
        states[:, i] = da.step(*controller(da.state, i))

    plot_trajectory(states)
    print('done')