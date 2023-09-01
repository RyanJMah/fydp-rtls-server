# Python Implementation of Standard (Discrete) Kalman Filter using numpy
# The implementation of the Fitler is based on the following paper:
# http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf

import numpy as np 
from numpy.linalg import multi_dot
from numpy.linalg import inv   # pinv may be more generic 

from gl_conf import GL_CONF

class KalmanFilter():
    def __init__ (self, A=None, H=None, obj_id=None):        
        if(A is None or obj_id is None):
            raise ValueError("State Model(A), Transition (H) Matric, and object ID must be provided in object creation!")            

        self.n_states = A.shape[1]      # no. of column vectors in A (length of the state vector)
        self.m_outputs = H.shape[0]     # no. of row vectors in H (length of the meas. vector)
        
        # Set the place holders of Parameters for KF when object creation 
        self.A = A
        self.H = H
        self.B = 0 
        self.Q = np.eye(self.n_states) 
        self.R = np.eye(self.n_states) 
        
        self.P_p = np.eye(self.n_states)           # prior before meas. update
        self.P_m = np.eye(self.n_states)           # posterior after meas. update
        self.x_p = np.zeros((self.n_states, 1))    # prior 
        self.x_m = np.zeros((self.n_states, 1))    # posterior 

        self.id = 0 if obj_id is None else obj_id  # id to differentiate multiple KF objects in application 
        self.isKalmanInitialized = False           # Set a flag for kalman initialization 


    # Initialize the State vector for pior and posterior
    def initState (self, x_0): 
        self.x_p = x_0      # Prior state 
        self.x_m = x_0      # Current state after measurement update 


    # Initialize the Covariance matrices for prior and posterior
    def initStateCovariance (self, P0):
        self.P_p = P0    # prior cov.
        self.P_m = P0    # current cov. after measurement update
    

    # Initialize the system parameters for KF 
    def assignSystemParameters(self, A = None, B = None, H = None, Q = None, R = None, P=None, x_0=None): 
        if(A is None or H is None):
            raise ValueError("State Model(A) and Transition Matric must be provided in KF!")            

        self.n_states = A.shape[1]      # no. of column vectors in A (length of the state vector)
        self.m_outputs = H.shape[0]     # no. of row vectors in H (lenght of the meas. vector)

        self.A = A
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n_states) if Q is None else Q
        self.R = np.eye(self.n_states) if R is None else R 

        self.P_p = np.eye(self.n_states) if P is None else P             # prior before meas. update
        self.P_m = np.eye(self.n_states) if P is None else P             # posterior after meas. update
        self.x_p = np.zeros((self.n_states, 1)) if x_0 is None else x_0  # prior 
        self.x_m = np.zeros((self.n_states, 1)) if x_0 is None else x_0  # posterior 


    # Evaluate the KF based on the measurement (z) and intput (u) data
    def performKalmanFilter(self, z, u): 
        # Time Update
        self.x_p = np.dot(self.A, self.x_m) + np.dot(self.B, u)
        self.P_p = multi_dot([self.A, self.P_m, np.transpose(self.A) ]) + self.Q

        # Measurement Update
        S = multi_dot([self.H, self.P_p, np.transpose(self.H)]) + self.R
        K = multi_dot([self.P_p, np.transpose(self.H), inv(S)])   # Kalman Gain
        z_residual = z - np.dot(self.H, self.x_p)                 # Diff b/w measurement and prior
        self.x_m = self.x_p + np.dot(K, z_residual)
        self.P_m = self.P_p - multi_dot([K, self.H, self.P_p]) 


# Initialize the Constant Velocity model for UWB-based Positioning in KF 
def initConstVelocityKF():
    dt = GL_CONF.update_period_secs
    
    # variance of the measurement noise, supposing the error is 15 cm (0.15 * 0.15) on X- & Y-axes
    # Tune the values based on your set-up and the demanded performance
    v_n = np.array([ 0.0225, 0.0225,  0.08])
    sigma_sq = 0.01    # Process noise value based on 10cm precision
    
    # Initial assumed values of the state which includes pose and their corresponding velocites
    x_0 = np.array([1.2, 1.82 , 0.885 , 1, 2, 1.5])  #  Initial guest
    x_0.shape = (len(x_0),1)      # force to be a column vector (6,1)
    
    # Initial value of State Covariance Matrix or Error Covariance 
    P_0 = np.multiply(2, np.eye(6))
    
    # State Model or Transition Matrix 
    A = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    
    B = 0 # No control input
    
    #  Relation b/w the observed Measurement and the State vector
    H = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0]])
    
    # Measurement Noise Covariance
    R = np.array([[v_n[0], 0, 0],
                  [0, v_n[1], 0],
                  [0, 0, v_n[2]]])
    
    '''
    Process Noise Covariance
    Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
    Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
    Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
    '''
    q_uD = (np.power(dt, 4))/4   # (dt, 3)/3  in some lit.
    q_oD = (np.power(dt, 3))/2   # (dt, 2)/2  in some lit.
    q_lD = np.square(dt)         # dt         in some lit.
    
    Q_0 = np.array([[q_uD, 0, 0, q_oD, 0, 0],
                    [0, q_uD, 0, 0, q_oD, 0],
                    [0, 0, q_uD, 0, 0, q_oD],
                    [q_oD, 0, 0, q_lD, 0, 0],
                    [0, q_oD, 0, 0, q_lD, 0],
                    [0, 0, q_oD, 0, 0, q_lD]])

    Q = Q_0 * sigma_sq
    
    return A, B, H, Q, R, P_0, x_0 


# Constant acceleration Motion Model for UWB-based Positioning in KF 
def initConstAccelerationKF():
    dt = GL_CONF.update_period_secs
    
    # variance of the measurement noise, supposing the error is 15 cm (0.15 * 0.15) on X- & Y-axes
    # Tune the values based on your set-up and the demanded performance
    v_n = np.array([ 0.0225, 0.0225,  0.08])
    sigma_sq = 0.01    # Process noise value based on 10cm precision

    # Initial guest for state vector 
    x_0 = np.array([1.2, 1.82 , 0.885 , 1, 2, 1.5, 0.5, 1.0, 0.75])  #  Initial guest
    x_0.shape = (len(x_0),1)      # force to be a column vector  (9, 1)

    # Initial guese of State Covariance Matrix shoudn't be zeros 
    P_0 = np.multiply(2, np.eye(len(x_0)))  # (9x9)


    # State Model or Transition Matrix  (9x9)
    A_oD = (np.power(dt, 2))/2  # coefficient for acceleration (off-diagonal)
    A = np.array([[1, 0, 0, dt, 0, 0, A_oD, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, A_oD, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0, A_oD],
                  [0, 0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, dt, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    B = 0 # No control input
    
    #  Relation b/w the observed Measurement and the State vector (3x9)
    H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0]])
    
    # Measurement Noise Covariance (3x3) 
    R = np.array([[v_n[0], 0, 0],
                  [0, v_n[1], 0],
                  [0, 0, v_n[2]]])
    
    '''
    Process Noise Covariance (9x9)
    Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
    Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
    Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
    '''
    q_t4 = (np.power(dt, 4))/4   # (dt, 3)/3  in some lit.
    q_t3 = (np.power(dt, 3))/2   # (dt, 2)/2  in some lit.
    q_t2 = np.square(dt)         # dt         in some lit.
    
    Q_0 = np.array([[q_t4, 0, 0, q_t3, 0, 0, q_t2, 0, 0],
                    [0, q_t4, 0, 0, q_t3, 0, 0, q_t2, 0],
                    [0, 0, q_t4, 0, 0, q_t3, 0, 0, q_t2],
                    [q_t3, 0, 0, q_t2, 0, 0, dt, 0, 0],
                    [0, q_t3, 0, 0, q_t2, 0, 0, dt, 0],
                    [0, 0, q_t3, 0, 0, q_t2, 0, 0, dt],
                    [q_t2, 0, 0, dt, 0, 0, 1, 0, 0],
                    [0, q_t2, 0, 0, dt, 0, 0, 1, 0],
                    [0, 0, q_t2, 0, 0, dt, 0, 0, 1]])

    Q = Q_0 * sigma_sq
    
    return A, B, H, Q, R, P_0, x_0 
