import numpy as np

def inv_transformation_matrix(E):
    R = E[:3, :3]
    T = E[:3, -1].reshape((3,1))
    assert R.shape == (3,3) and T.shape == (3, 1)
    return np.hstack((R.T,-R.T@T))
    