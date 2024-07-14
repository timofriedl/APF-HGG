import ctypes

import numpy as np
import time

lib = ctypes.CDLL('./apf/build/src/libapf_hgg_core.so')

lib.step.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double theta[8]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double targetPosValues[3]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double *obstacleAttributes
    ctypes.c_size_t,  # size_t obstacleCount
    ctypes.c_double,  # double dt
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double integralValues[8]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double prevErrorValues[ACT_COUNT]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1)  # double forceResult[ACT_COUNT]
]

theta64 = np.zeros(7, dtype=np.float64)
target_pos_values64 = np.zeros(3, dtype=np.float64)

integral_values = np.zeros(8, dtype=np.float64)
prev_error_values = np.zeros(8, dtype=np.float64)
prev_time = -1


def control_step(theta: np.ndarray, target_pos_values: np.ndarray, obstacle_attributes: np.ndarray):
    assert theta.shape[0] == 7
    assert target_pos_values.shape[0] == 3

    global prev_time
    now = time.time()
    dt = 0.001 if prev_time < 0 else 1.0 / (now - prev_time)
    prev_time = now

    theta64[:] = theta
    target_pos_values64[:] = target_pos_values[:]

    force_result = np.zeros(7, dtype=np.float64)
    lib.step(theta64, target_pos_values64, obstacle_attributes.flatten(), len(obstacle_attributes), dt,
             integral_values, prev_error_values, force_result)

    return force_result


if __name__ == '__main__':
    print("Trying the APF-HGG C++ library...")

    theta_values = np.zeros(7, dtype=np.float32)
    target_pos = np.zeros(3, dtype=np.float32)

    attr = [
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7],
        [3, 4, 5, 6, 7, 8],
        [4, 5, 6, 7, 8, 9]
    ]
    obs_attr = np.array(attr, dtype=np.float64)

    force = control_step(theta_values, target_pos, obs_attr)
    duration = time.time() - prev_time
    print(force)
    print("{} Hz".format(1.0 / duration))
