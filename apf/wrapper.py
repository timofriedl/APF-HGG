import ctypes

import numpy as np
import time

lib = ctypes.CDLL('./apf/build/src/libapf_hgg_core.so')

lib.apf.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double q[8]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double targetPosValues[3]
    ctypes.c_double,  # double targetGripper
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double *obstacleAttributes
    ctypes.c_size_t,  # size_t obstacleCount
    ctypes.c_double,  # double dt
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double integralValues[8]
    ctypes.c_double,  # double prevErrorValues[ACT_COUNT]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1)  # double forceResult[ACT_COUNT]
]

q64 = np.zeros(8, dtype=float64)

integralValues = np.zeros(8, dtype=float64)
prevErrorValues = np.zeros(8, dtype=float64)
lastTime = None

def control_step(q: np.ndarray, target_pos_values: np.ndarray, target_gripper, obstacle_attributes: np.ndarray):
    global lastTime
    now = time.time()
    dt = 0.001 if lastTime is None else 1.0 / (now - lastTime)
    lastTime = now

    q64[:] = q[:]

    force_result = np.zeros(8, dtype=float64)
    lib.apf(q64, target_pos_values, target_gripper, obstacle_attributes, len(obstacle_attributes), dt, integralValues, prevErrorValues, force_result)
    return force_result
