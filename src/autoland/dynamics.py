import numpy as np
from numpy import sin, cos, tan

class DubinsAirplane:
    '''
        Dubin's Airplane model based on:
        Owen, Mark et. al. "Implementing Dubins Airplane Paths on Fixed-Wing UAVs"

        https://link.springer.com/referenceworkentry/10.1007/978-90-481-9707-1_120
    '''

    def __init__(self, dt=0.05):
        self._dt = dt
        self._g = 9.8  # Down gravity  (m/s^2)
        # state is relative to a point in the sky above the runway
        self._rn  = 0. # North         (m)
        self._re  = 0. # East          (m)
        self._rd  = 0. # Down          (m)
        self._psi = 0. # Heading Angle (rad)

    @property
    def state(self):
        return (self._rn, self._re, self._rd, self._psi)

    def step(self, V, gamma, phi):
        '''
            Dynamics assume low-level flight controller and small enough dT
            that controls are instantaneously respected

            V     := commanded airspeed          (m/s)
            gamma := commanded flight path angle (rad)
            phi   := commanded bank angle        (rad)
        '''

        rn_dot  = V * cos(self._psi) * cos(gamma)
        re_dot  = V * sin(self._psi) * cos(gamma)
        rd_dot  = -V * sin(gamma)
        psi_dot = (self._g / V) * tan(phi)

        self._rn  += rn_dot  * self._dt
        self._re  += re_dot  * self._dt
        self._rd  += rd_dot  * self._dt
        self._psi += psi_dot * self._dt

        return self.state

