from autoland.pid import PID
import math

class GlideSlopeController:
    def __init__(self, client, des_u, des_psi, h_thresh, gamma, dt=0.1):
        self._client   = client
        self._des_u    = des_u
        self._des_psi  = des_psi
        self._h_thresh = h_thresh # height of runway threshold
        self._gamma    = gamma    # glide slope angle

        self._tan_gamma = math.tan(math.radians(self._gamma))

        # PI controllers
        self._psi_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._y_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._phi_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._u_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._theta_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._h_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._thrust_pid = PID(dt, kp=1., ki=1., kd=0.)
        self._elev_pid = PID(dt, kp=1., ki=1., kd=0.)


    def control(self, statevec):
        '''
        INPUTS
            Statevector based on https://arc.aiaa.org/doi/10.2514/6.2021-0998
            statevec with components
                u      - longitudinal velocity (m/s)
                v      - lateral velocity (m/s)
                w      - vertical velocity (m/s)
                p      - roll velocity (deg/s)
                q      - pitch velocity (deg/s)
                r      - yaw velocity (deg/s)
                phi    - roll angle (deg)
                theta  - pitch angle (deg)
                psi    - yaw angle (deg)
                x      - horizontal distance (m)
                y      - lateral deviation (m)
                h      - aircraft altitude (m)
        OUTPUTS
            throttle
            elevator
            rudder
            aileron
        '''

        u, v, w, \
        p, q, r, \
        phi, theta, psi, \
        x, y, h = statevec

        # lateral control
        err_y = y
        err_psi = psi - self._des_psi
        err_phi = phi

        # longitudinal control
        err_u = u - self._des_u
        h_c = self._h_thresh + x*self._tan_gamma
        err_h = h - h_c
        theta_c = self._theta_pid(err_h)
        err_theta = theta - theta_c

        # then saturate

        # then return
        raise NotImplementedError('Method not completed')