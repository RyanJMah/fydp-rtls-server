class LowPassFilter:
    """
    First order low pass filter, discretized via trapazoidal rule,
    with a sample time of 5.5Hz
    """

    def __init__(self, tau: float):
        self.tau = tau

        self.y_n_minus_1 = 0
        self.u_n_minus_1 = 0

    def exec(self, u):
        y = ( 1/(11*self.tau + 1) ) * \
            (u + self.u_n_minus_1 - (1 - 11*self.tau)*self.y_n_minus_1)

        self.y_n_minus_1 = y
        self.u_n_minus_1 = u

        return y
