import numpy as np
import numpy.linalg as lin
import scipy.spatial.transform as T

# Camera parameters
i_x = -0.500  # m
i_y = -0.160  # m
i_z = -1.140  # m

i_roll  = (-99.8) #deg
i_pitch = (0)  #deg
i_yaw   = (-90.9)  #deg

res_x = 1280 #px
res_y = 1080 #px

c_x = 636.0 #px
c_y = 548.0 #px
f_x = 241.0 #px
f_y = 238.0 #px

u = 795 #px
v = 467 #px
d = 2.7 #m


def extrinsic_matrix(x,y,z,roll,pitch,yaw):
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    quat_roll  = T.Rotation.from_quat([np.sin(roll / 2.0), 0,                   0              , np.cos(roll / 2.0)  ]) # X
    quat_pitch = T.Rotation.from_quat([0,                  np.sin(pitch / 2.0), 0              , np.cos(pitch / 2.0) ]) # Y
    quat_yaw   = T.Rotation.from_quat([0,                  0,                   np.sin(yaw/2.0), np.cos(yaw / 2.0)   ]) # Z      
    
    # Intrinsic Rotation z'y'x'' = ZYX
    rot = quat_roll * quat_pitch * quat_yaw
    rot = rot.as_matrix()

    translation = np.zeros((4,4))
    translation[:3, :3] = rot
    
    translation[0, 3] = y
    translation[1, 3] = z
    translation[2, 3] = x
    translation[3, 3] = 1

    return translation

def intrinsic_matrix():
    intrinsic = np.array([
        [f_x, 0,   c_x, 0],
        [0,   f_y, c_y, 0],
        [0,   0,   1,   0],
        [0,   0,   0,   1]
    ])
    
    return intrinsic
    

question = np.array([[u],[v],[1],[1]])

intrinsic = intrinsic_matrix()
extrinsic = extrinsic_matrix(i_x,i_y,i_z,i_roll,i_pitch,i_yaw)

proj = intrinsic @ extrinsic
inv_proj = lin.inv(proj)

unproj_question =  inv_proj @ question
print("SOLUTION:")
print(unproj_question * d)