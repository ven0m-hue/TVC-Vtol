
#!/usr/bin/env python
# coding: utf-8
from imu import *
import numpy as np 
from time import time,sleep 

# function to convert from euler to quaternion 
# rad to degree is pending 
def euler2quat(roll, pitch, yaw):
    
    #Short hand
    croll, sroll   = np.around((np.cos(roll/2)), decimals=15) , np.around((np.sin(roll/2)), decimals=15)
    cpitch, spitch = np.around((np.cos(pitch/2)), decimals=15), np.around((np.sin(pitch/2)), decimals=15)
    cyaw, syaw     = np.around((np.cos(yaw/2)), decimals=15)  , np.around((np.sin(yaw/2)), decimals=15)
    
    #Quaternion variables 
    w = croll * cpitch * cyaw + sroll * spitch * syaw
    x = sroll * cpitch * cyaw - croll * spitch * syaw
    y = croll * spitch * cyaw + sroll * cpitch * syaw
    z = croll * cpitch * syaw - sroll * spitch * cyaw
    
    return w, x, y, z
    


# function to convert from quat to euler 
def quat2euler(q):
    
    # rotatation matrix interms of quaternions 
    R = np.array([[ 1 - 2 * (q[3] * q[3]+q[2] * q[2]),      2 * (q[1] * q[2]-q[0] * q[3]),      2 * (q[1] * q[3]+q[0] * q[2])],
                [2 * (q[1] * q[2]+q[0] * q[3]),       1 - 2 * (q[3] * q[3]+q[1] * q[1]),      2 * (q[2] * q[3]-q[0] * q[1])],
                [2 * (q[1] * q[3]-q[0] * q[2]),      2 * (q[2] * q[3]+q[0] * q[1]),       1 - 2 * (q[2] * q[1]+q[2] * q[2])]
                 ])
    roll  = np.degrees(np.arctan2(R[2,1],R[2,2]))
    pitch = np.degrees(-np.arcsin(R[2,0]))
    yaw   = np.degrees(np.arctan2(R[1,0],R[0,0]))
    
    return roll, pitch, yaw 


# measurement function 
# This function measures the accel and mag in terms of euler and feeds to the measurement matrix Z
def measure_acc_mag(accel_data, mag_data):
    
    ax, ay, az = accel_data/np.linalg.norm(accel_data)
    mx, my, mz = mag_data/np.linalg.norm(mag_data)  * 0   # Mag is not calibrated at the moment. Unclibrated mag can seriously affect the roll and pitch.
                                                      # For now, just using roll and pitch. Mag can be easily neglected as it acounts only for the yaw motion.
                                                      # Future update, requires a calibrated mag value. To accuurately estimate the yaw.
    roll, pitch = np.arctan2(-ay,-az), np.arctan2(ax, np.sqrt(ay**2 + az**2))
    
    mx = mx * np.cos(pitch) + my * np.sin(roll) * np.sin(pitch) + mz * np.cos(roll) * np.sin(pitch)
    my = my * np.cos(roll) - mz * np.sin(roll);
    
    yaw = np.arctan2(-my, mx)
    yaw = yaw + 2*np.pi if yaw < 0 else yaw 
    
    return roll, pitch, yaw


def self_calibrate_accel(N=100):
    roll_offset, pitch_offset = 0.0, 0.0
    
    for i in range(N):
        roll, pitch = imu.get_acc_angles()
        roll_offset += roll
        pitch_offset += pitch
        sleep(.01)
    roll_offset = float(roll_offset) / float(N)
    pitch_offset = float(pitch_offset) / float(N)
    print("Accelerometer offsets: " + str(roll_offset) + "," + str(pitch_offset))
    sleep(2)
    return(roll_offset, pitch_offset)

# All the matrix inits
# Noise Covariance MATRIX
# Process Covariance Matrix Q
w_noise = 1e-6 * np.eye(4,7)
Q = np.eye(7,7) # auxillary 
w_bias = 1e-8 * Q[4:]
Q = np.concatenate((w_noise,w_bias))

# Measuremnt Covariance Matrix
gyro_bias = np.array([.1, .1, .1]) # Gyro bias, after calibration 
'''
acc_noise = 1e-1 * np.eye(3,6)
R = np.eye(6,6) # auxillary
mag_noise = 2e-1 * R[3:]
R = np.concatenate((acc_noise,mag_noise))
'''
# For now  R
acc_noise = 1e-1 * np.eye(3,3)
R = acc_noise

# Estimate Covarinace Matrix P
P = np.eye(7,7)


# From IMU get the raw data
imu = MPU9250()
roll_offset, pitch_offset = self_calibrate_accel(N=100) # accel calibration
accel_data = imu.get_accel()
mag_data   = imu.get_mag()

print(accel_data)
print(imu.get_acc_angles())

roll, pitch, yaw = measure_acc_mag(accel_data, mag_data)
print("Printing the iniital set of Euler's RPY data to compare, test Falg")
print("{0:.6f}".format(roll  * 180/ np.pi))
print("{0:.6f}".format(pitch * 180/ np.pi))
print("{0:.2f}".format( 360 - (yaw) * 180/ np.pi))
print("-"*50)

#init the state vector 
quat_init  = euler2quat(roll,pitch,yaw)  # init the quaternions
state_vec = np.concatenate((quat_init,gyro_bias))
print('Quaternions initialized')
print(state_vec)
print("-"*50)

[roll_init, pitch_init, yaw_init] = quat2euler(state_vec[:4])
# Measuremnt Matrix init
print("Euler initialized")
print(roll_init, pitch_init, yaw_init)
quat_init  = euler2quat(roll,pitch,yaw)
print(quat_init)

# Sampling time using the ras pi
roll_offset, pitch_offset = self_calibrate_accel(N=100) # accel calibration
sleep(1)
start_time = time()

# Super loop 
while True: 
    dt = time() - start_time   # if this does not give a good result, then manually caluclate the desired samplling time 
    start_time = time()

    # Call the accel data
    accel_data = imu.get_accel() 
    ax, ay, az = accel_data/np.linalg.norm(accel_data)  # for now 
    Z = np.array([ax, ay, az])

    # Gyro bias modelling 
    gyro_current_x = state_vec[4]
    gyro_current_y = state_vec[5]
    gyro_current_z = state_vec[6]

    # True values, according to our initial assumtion
    # from the get_gyro function 
    gx, gy, gz = imu.get_gyro()
    gx -= gyro_current_x
    gy -= gyro_current_y
    gz -= gyro_current_z

    omega = np.array([[0  , -gx, -gy, -gz],
                      [gx ,   0,  gz, -gy],
                      [gy , -gz,   0,  gx],
                      [gz ,  gy,  -gx,  0]])


    # sensor frame refered to the earth fframe 
    quat = state_vec[:4]
    Ts = dt                # Sampling Period 
    
    q_new = quat + 0.5 * Ts * omega.dot(quat)


    #Normalize 
    gyro_new_bias = np.array([gyro_current_x,gyro_current_y,gyro_current_z])
    q_norm = q_new/ np.linalg.norm(q_new)
    new_state_vec = np.concatenate([q_norm,gyro_new_bias])
   
   
    # State trnasistion vector 
    line = np.array([[ quat[1], quat[2], quat[3]],
                 [-quat[0], quat[3],-quat[2]],
                 [-quat[3],-quat[0], quat[1]],         
                 [ quat[2],-quat[1],-quat[0]]
                ])

    OnL = np.concatenate([omega,line.reshape(4,3)], axis=1)

    #Jacobian to liearize the model     
    Ak  = np.concatenate([OnL, np.zeros([3,7])]) # (7,7)

    # Linearized state transistion model   
    F = np.eye(7) + 0.5 * Ts * Ak

    # Computing the error covariance martrix 
    P_next = F.dot(P).dot(np.transpose(F)) + Q

    '''
     Fusion block 

     Jacobian Observation Matrix 
     For now only accel data is considered, mag data are extracted
     seperatley 

    '''
    # State Observation Matrix 
    Hk = 2 * np.array([[  new_state_vec[2], -new_state_vec[3],  new_state_vec[0], -new_state_vec[1]],
                       [ -new_state_vec[1], -new_state_vec[0], -new_state_vec[3], -new_state_vec[2]],
                       [ -new_state_vec[0],  new_state_vec[1],  new_state_vec[2], -new_state_vec[3]]
                     ])

    H = np.concatenate([Hk.reshape(3,4), np.zeros([3,3])], axis=1)



    #Computing the KALMAN GAIN
    #Auxillary
    Sp = H.dot(P_next).dot(np.transpose(H)) + R 

    # Kalman gain 
    K = P_next.dot(np.transpose(H)).dot(np.linalg.inv(Sp))

    # Output
    X_ = new_state_vec[:4]

    # Observation model 
    hk = np.array([-2 *(X_[1]*X_[3]     -       X_[0]*X_[2]),
              -2*(X_[0]*X_[1]       +       X_[2]*X_[3]),
              -1*(X_[0]**2+X_[3]**2 - X_[1]**2-X_[2]**2)
             ])
    #Error variance
    S = Z.reshape(3,1) - hk.reshape(3,1) 

    #Predicted state Matrix 
    state_vec = new_state_vec.reshape(7,1) + K.dot(S)


    # Normalize 
    gyro_new_bias = (np.array([gyro_current_x,gyro_current_y,gyro_current_z])).reshape(3,1)
    auxillary = state_vec[:4]/ np.linalg.norm(state_vec)
    state_vec = np.concatenate([auxillary,gyro_new_bias])
    state_vec = state_vec.reshape(7,)

    save_me = np.eye(7) - K.dot(H) #Auxillary, for some reason directly conputing the equation ->  P = (np.eye(7) - K.dot(H)).dot(P_next) , gave NaN error. 
    P = (save_me).dot(P_next)
   
    # extract quaternions and convert them to RPY (NED)
    q_estimate = state_vec[:4]

    # And finally, the RPY
    estimate_roll, estimate_pitch, estimate_yaw = quat2euler(q_estimate)
    estimate_roll -= roll_offset 
    estimate_pitch -= pitch_offset 
    estimate_roll, estimate_pitch = np.around(estimate_roll, decimals=3), np.around(estimate_pitch, decimals=3)
   
    print('Angles[theta] : {0:.2f}, {1:.2f}'.format(np.around(estimate_roll, decimals=3), np.around(estimate_pitch, decimals=3)))
    print("*"*90)

