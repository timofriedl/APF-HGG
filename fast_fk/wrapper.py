import ctypes

import numpy as np

lib = ctypes.CDLL('./fast_fk/build/lib.linux-x86_64-3.7/forward_kinematics.cpython-37m-x86_64-linux-gnu.so')

lib.forward_kinematics.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1),  # q, a float* of size (K * 7)
    ctypes.c_size_t,  # size
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1)  # result, a float* of size (K * 4 * 4)
]


def forward_kinematics(q: np.ndarray):
    k = len(q)
    q = q.flatten()
    result = np.empty(k * 4 * 4, dtype=np.float32)
    lib.forward_kinematics(q, k, result)
    return np.transpose(result.reshape(k, 4, 4), axes=(0, 2, 1))
