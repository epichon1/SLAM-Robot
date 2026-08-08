import numpy as np
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R

### Transform utilities

def tf_inv(transform):
    tf_inv = Transform()
    t = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    rmat = R.from_quat(np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])).as_matrix()
    rmat_inv = rmat.T
    quat_inv = R.from_matrix(rmat_inv).as_quat()
    t_inv = np.matmul(-rmat_inv, t)
    tf_inv.translation.x = t_inv[0]
    tf_inv.translation.y = t_inv[1]
    tf_inv.translation.z = t_inv[2]
    tf_inv.rotation.x = quat_inv[0]
    tf_inv.rotation.y = quat_inv[1]
    tf_inv.rotation.z = quat_inv[2]
    tf_inv.rotation.w = quat_inv[3]
    return tf_inv

def tf_mult(tf1, tf2):
    tf_mult = Transform()
    t1 = np.array([tf1.translation.x, tf1.translation.y, tf1.translation.z])
    rmat1 = R.from_quat(np.array([tf1.rotation.x, tf1.rotation.y, tf1.rotation.z, tf1.rotation.w])).as_matrix()
    t2 = np.array([tf2.translation.x, tf2.translation.y, tf2.translation.z])
    rmat2 = R.from_quat(np.array([tf2.rotation.x, tf2.rotation.y, tf2.rotation.z, tf2.rotation.w])).as_matrix()

    tmult = t1 + np.matmul(rmat1, t2)
    rmult = np.matmul(rmat1, rmat2)
    quatmult = R.from_matrix(rmult).as_quat()
    tf_mult.translation.x = tmult[0]
    tf_mult.translation.y = tmult[1]
    tf_mult.translation.z = tmult[2]
    tf_mult.rotation.x = quatmult[0]
    tf_mult.rotation.y = quatmult[1]
    tf_mult.rotation.z = quatmult[2]
    tf_mult.rotation.w = quatmult[3]

    return tf_mult
