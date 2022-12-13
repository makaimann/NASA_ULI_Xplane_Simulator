from dynamics import DubinsAirplane
import math
import numpy as np
import os
import sys
import time
from visualize import plot_trajectory

NASA_ULI_ROOT_DIR = os.environ['NASA_ULI_ROOT_DIR']
XPC3_DIR = os.path.join(NASA_ULI_ROOT_DIR, "src")
sys.path.append(XPC3_DIR)

from xpc3 import XPlaneConnect
from xpc3_helper import reset, homeToLocal


def frange(start, stop, dt):
    start = float(start)
    stop = float(stop)

    idx = start
    while idx <= stop:
        yield idx
        idx += dt


def setHomeState(client, x, y, z, heading, pitch, roll):
    """Set the aircraft's state using coordinates in the home frame.
    This is equivalent to setting the crosstrack error (x), downtrack
    position (y), and heading error (theta).

        Args:
            client: XPlane Client
            x: desired crosstrack error [-10, 10]      (meters)
            y: desired downtrack position [0, 2982]    (meters)
            z: amount below start AGL                  (meters)
            heading: desired heading error [-180, 180] (degrees)
            pitch:   pitch off of level plane          (degrees)
            roll:    roll of the aircraft              (degrees)
                     in NED where N is aligned with the runway
                     positive rolls counterclockwise around N axis
    """

    localx, localz = homeToLocal(x, y)

    client.sendDREF("sim/flightmodel/position/local_x", localx)
    client.sendDREF("sim/flightmodel/position/local_z", localz)
    client.sendDREF("sim/flightmodel/position/psi", 53.7 - heading)
    client.sendDREF("sim/flightmodel/position/theta", pitch)
    # roll is negated so that it's counterclockwise
    client.sendDREF("sim/flightmodel/position/phi", -roll)

    # TODO: move away from AGL
    #       ground is not perfectly flat so get strange bumps
    startAGL = 1000 # m; initial above ground level
    curr_agly = client.getDREF("sim/flightmodel/position/y_agl")[0]
    curr_localy = client.getDREF("sim/flightmodel/position/local_y")[0]
    new_agl = curr_localy - curr_agly + startAGL - z
    client.sendDREF("sim/flightmodel/position/local_y",
                    new_agl)

    if curr_agly < 0.0:
        client.pauseSim(False)
        raise RuntimeError("CRASH!")


if __name__ == '__main__':
    MAX_TIME = 200.0
    dt = 0.05

    da = DubinsAirplane(dt=dt)
    client = XPlaneConnect()

    def controller(state, t):
        V     = 70.0
        gamma = -0.2
        phi   = 0.0
        return V, gamma, phi

    reset(client)

    # in NED, but aligned with local coordinate frame
    # so that North is along the runway
    n, e, d, h = da.state
    setHomeState(client, e, n, d, math.degrees(h), 0.0, 0.0)
    last = time.time()
    for t in frange(0.0, MAX_TIME, dt):
        V, gamma, phi = controller(da.state, t)
        da.step(V, gamma, phi)
        n, e, d, h = da.state
        setHomeState(client, e, n, d, math.degrees(h), math.degrees(gamma), math.degrees(phi))
        elapsed = time.time() - last
        if elapsed < dt:
            time.sleep(dt-elapsed)
        last = time.time()


    print('done')