import numpy as np
from numpy.typing import NDArray
from typing import List, Tuple, Callable

import logs

logger = logs.init_logger(__name__)


def linearized_lse(A: NDArray, b: NDArray) -> NDArray:
    """
    Least squares estimate of a system of equations is A^T * A * x = A^T * b
        - Refence: https://textbooks.math.gatech.edu/ila/least-squares.html
    """

    tmp = np.linalg.inv( np.matmul(A.T, A) )
    tmp = np.matmul(tmp, A.T)
    x   = np.matmul(tmp, b)

    return x

# Reference for Gauss-Newton method:
#   - https://en.wikipedia.org/wiki/Gauss–Newton_algorithm
#
# Implementation based off:
#   - https://github.com/basil-conto/gauss-newton/blob/master/gaussnewton.py

def gauss_newton_lse( r  : Callable[ [NDArray], NDArray ],      # R^n -> R^m
                      J  : Callable[ [NDArray], NDArray ],      # R^n -> R^(m x n)
                      x0 : NDArray,                             # R^n
                      n  : int,                                 # Number of variables
                      m  : int,                                 # Number of equations
                      tolerance: float = 1E-3,
                      max_iters: int = 200 ) -> Tuple[int, NDArray]:
    """
    x -> initial guess, R^n

    r -> residual function, vector of m equations, r(x) = (r1, r2, ..., r_m), r_i: R^n -> R

    J -> Jacobian of r, J(x) = partial(r_i)/partial(x_j), J: R^n -> R^(m x n)
    """

    # Determine which inverse function to use, inv is faster than pinv,
    # so we want to use inv if possible
    inv_func: Callable

    if (m > n):
        inv_func = np.linalg.pinv   # Moore-Penrose pseudo-inverse for overdetermined systems

    elif (m == n):
        inv_func = np.linalg.inv    # Can just do regular inverse for m = n

    else:
        logger.error("Passed underdetermined system to gauss_newton_lse, exiting function...")
        return (0, x0)


    dx = np.ones(len(x0))   # Correction vector
    xn = np.array(x0)       # Approximation of solution

    i = 0
    while (i < max_iters) and (dx[dx > tolerance].size > 0):
        dx  = np.dot( inv_func( J(xn) ), -r(xn) )
        xn += dx            # x_{n + 1} = x_n + dx_n
        i  += 1

    return (i, xn)
