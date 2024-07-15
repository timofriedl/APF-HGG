import ctypes

import numpy as np
import time

JOINT_COUNT = 7

lib = ctypes.CDLL('./apf/build/src/libapf_hgg_core.so')

lib.step.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double theta[8]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double targetPosValues[3]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double targetRotValues[4]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double *obstacleAttributes
    ctypes.c_size_t,  # size_t obstacleCount
    ctypes.c_double,  # double dt
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double integralValues[8]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),  # double prevErrorValues[ACT_COUNT]
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1)  # double forceResult[ACT_COUNT]
]

theta64 = np.zeros(JOINT_COUNT, dtype=np.float64)
target_pos_values64 = np.zeros(3, dtype=np.float64)
target_rot_values64 = np.zeros(4, dtype=np.float64)

integral_values = np.zeros(JOINT_COUNT, dtype=np.float64)
prev_error_values = np.zeros(JOINT_COUNT, dtype=np.float64)


def control_step(theta: np.ndarray, target_pos_values: np.ndarray, target_rot_values: np.ndarray,
                 obstacle_attributes: np.ndarray, dt):
    assert theta.shape[0] == JOINT_COUNT
    assert target_pos_values.shape == (3,)
    assert target_rot_values.shape == (4,)
    if len(obstacle_attributes) > 0:
        assert obstacle_attributes.ndim == 2
        assert obstacle_attributes.shape[1] == 7

    theta64[:] = theta
    target_pos_values64[:] = target_pos_values[:]
    target_rot_values64[:] = target_rot_values[:]

    force_result = np.zeros(JOINT_COUNT, dtype=np.float64)
    lib.step(theta64, target_pos_values64, target_rot_values64, obstacle_attributes.flatten(),
             obstacle_attributes.shape[0], dt,
             integral_values, prev_error_values, force_result)

    return force_result


if __name__ == '__main__':
    print("Trying the APF-HGG C++ library...")

    np.random.seed = 42
    theta_values = np.random.rand(JOINT_COUNT)
    target_pos = np.zeros(3, dtype=np.float32)
    target_rot = np.array([0, 1, 0, 0], dtype=np.float32)

    attr = [
        [0, 1, 2, 3, 4, 5, 6],
        [1, 2, 3, 4, 5, 6, 7],
        [2, 3, 4, 5, 6, 7, 8],
        [3, 4, 5, 6, 7, 8, 9],
        [4, 5, 6, 7, 8, 9, 0]
    ]
    obs_attr = np.array(attr, dtype=np.float64)

    start_time = time.time()
    torques = control_step(theta_values, target_pos, target_rot, obs_attr, 0.001)
    duration = time.time() - start_time

    np.set_printoptions(precision=2)
    print("Output torques: {}".format(torques))
    print("Theoretical frequency: %d Hz" % (1.0 / duration))
