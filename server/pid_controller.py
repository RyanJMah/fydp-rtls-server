from gl_conf import GL_CONF

class PIDController:
    """
    Reference: https://www.youtube.com/watch?v=zOByx3Izf5U
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float, derivative_lpf_tau: float = 0.1):
        self.kp       = kp
        self.ki       = ki
        self.kd       = kd
        self.setpoint = setpoint

        self.T = 1 / GL_CONF.update_period_secs

        self.tau = derivative_lpf_tau

        self.p = 0
        self.i = 0
        self.d = 0

        self.prev_err = 0

    def exec(self, err: float):
        self.p = self.kp*err

        self.i = ( (self.ki*self.T)/2 )*( err - self.prev_err ) + self.i

        self.d = ( (2*self.kd)/(2*self.tau + self.T) )*( err - self.prev_err ) + ( (2*self.tau - self.T)/(2*self.tau + self.T) )*self.d

        return self.p + self.i + self.d
