import numpy as np

import matplotlib.pyplot as plt
from numpy import random

discrete_grid = np.zeros((200, 200, 36), dtype=float)

groundtruth_poses = []

#######################
### Initial Setting ###
#######################

pose = [0, 0, 0]    # x [m], y [m], heading [radian]
groundtruth_poses.append(pose)

print('Initial pose : {}'.format(pose))
plt.plot(pose[0], pose[1], 'o')
plt.arrow(pose[0], pose[1], 20 * np.cos(pose[2]), 20 * np.sin(pose[2]), width=1)

sample_odom_est = np.ndarray(shape=(2, 1),buffer=np.array([pose]))
for iteration in range(10):

    print('Iteration : {}'.format(iteration))

    dx = random.randint(low=10, high=220, size=1)[0]
    dy = random.randint(low=10, high=110, size=1)[0]
    dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]
    
    print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

    translation = np.sqrt(dx**2 + dy**2)
    rotation_1 = np.arctan2(dy, dx) - pose[2]
    rotation_2 = dtheta - rotation_1
    print('Translation : {}'.format(translation))
    print('rotation_1 : {}'.format(rotation_1))
    print('rotation_2 : {}'.format(rotation_2))

    # Rotation Noise
    rotation_est = np.random.normal(loc=rotation_1, scale=0.1 * abs(rotation_1), size=100)
    
    ####################################################################################
    # Gaussian Noise Accumulation
    idx_odom = 0
    for sample_odom in zip(sample_odom_est.T):
            
        idx_rotation = 0
        for rotation in zip(rotation_est):
            # print(rotation[0])
            # plt.plot(translation * np.cos(pose[2] + rotation[0]) + sample_odom[0][0], translation * np.sin(pose[2] + rotation[0]) + sample_odom[0][1], 'x', alpha=0.5)

            # For each noisy pose estimation, apply translation and sample Gaussian distriubtion.
            # Covariance of the distribution depends on the value of translation.
            # This is because longer the robot travels, its odometry will have more likelihood to receive noise.
            mean = [translation * np.cos(pose[2] + rotation[0]) + sample_odom[0][0], translation * np.sin(pose[2] + rotation[0]) + sample_odom[0][1]]
            cov = [[translation, 0], 
                    [0, translation]]
            odom_est = np.random.multivariate_normal(mean, cov, 100).T

            # Accumulate all the odometry distribution values
            if idx_odom == 0:
                concat_array = odom_est
                idx_odom += 1
            else:
                concat_array = np.concatenate((concat_array, odom_est), axis=1)

    # Since it is not possible to use all the values in odometry distribution, acquire a sample from the distribution
    sample_idx_mask = np.random.randint(low=0, high=concat_array.shape[1], size=100)

    sample_odom_est = concat_array[:, sample_idx_mask]
    #####################################################################################

    pose = [pose[0] + dx, pose[1] + dy, pose[2] + dtheta]
    groundtruth_poses.append(pose)             
    print('Pose : {}'.format(pose))

    plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
    plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')
    plt.arrow(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 20 * np.cos(groundtruth_poses[iteration+1][2]), 20 * np.sin(groundtruth_poses[iteration+1][2]), width=1)

    plt.arrow(groundtruth_poses[iteration][0], groundtruth_poses[iteration][1], 20 * np.cos(groundtruth_poses[iteration][2] + rotation_1), 20 * np.sin(groundtruth_poses[iteration][2] + rotation_1), width=1, color='red')

    pose_est_x = np.mean(sample_odom_est[0])
    pose_est_y = np.mean(sample_odom_est[1])
    print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
    plt.plot(pose_est_x, pose_est_y, 'o')

plt.show()

##########################################
### Iterative Odometry Pose Estimation ###
##########################################
# for iteration in range(2):

#     print('Iteration : {}'.format(iteration))

#     # At the first iteration, acquire the odometry distribution from initial pose
#     if iteration == 0:

#         # Control Commands
#         dx = random.randint(low=10, high=220, size=1)[0]
#         dy = random.randint(low=10, high=110, size=1)[0]
#         dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]

#         print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

#         translation = np.sqrt(dx**2 + dy**2)
#         rotation_1 = np.arctan2(dy, dx) - pose[2]
#         rotation_2 = dtheta - rotation_1
#         print('Translation : {}'.format(translation))
#         print('rotation_1 : {}'.format(rotation_1))
#         print('rotation_2 : {}'.format(rotation_2))

#         # Rotation Noise
#         rotation_est = np.random.normal(loc=rotation_1, scale=0.1 * abs(rotation_1), size=100)
        
#         idx_rotation = 0
#         for rotation in zip(rotation_est):
#             # print(rotation[0])
#             # plt.plot(translation * np.cos(pose[2] + rotation[0]), translation * np.sin(pose[2] + rotation[0]), 'x')

#             mean = [translation * np.cos(rotation[0]), translation * np.sin(rotation[0])]
#             cov = [[translation, 0], 
#                 [0, translation]]
#             odom_est = np.random.multivariate_normal(mean, cov, 100).T

#             if idx_rotation == 0:
#                 concat_array = odom_est
#                 idx_rotation += 1
#             else:
#                 concat_array = np.concatenate((concat_array, odom_est), axis=1)
            
#             # plt.plot(sample_odom_est[0], sample_odom_est[1], 'x', alpha=0.1)

#         sample_idx_mask = np.random.randint(low=0, high=concat_array.shape[1], size=100)
            
#         sample_odom_est = concat_array[:, sample_idx_mask]
        
#         pose = [pose[0] + dx, pose[1] + dy, pose[2] + dtheta]
#         groundtruth_poses.append(pose)

#         print('Pose : {}'.format(pose))
    
#         plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
#         plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')
#         plt.arrow(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 20 * np.cos(groundtruth_poses[iteration+1][2]), 20 * np.sin(groundtruth_poses[iteration+1][2]), width=1)

#         pose_est_x = np.mean(sample_odom_est[0])
#         pose_est_y = np.mean(sample_odom_est[1])
#         print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
#         plt.plot(pose_est_x, pose_est_y, 'o')

#     # Iteratively acquire odometry distribution from each odometry estimation and accumulate it
#     else:
#         dx = random.randint(low=10, high=220, size=1)[0]
#         dy = random.randint(low=10, high=110, size=1)[0]
#         dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]
        
#         print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

#         translation = np.sqrt(dx**2 + dy**2)
#         rotation_1 = np.arctan2(dy, dx) - pose[2]
#         rotation_2 = dtheta - rotation_1
#         print('Translation : {}'.format(translation))
#         print('rotation_1 : {}'.format(rotation_1))
#         print('rotation_2 : {}'.format(rotation_2))

#         # Rotation Noise
#         rotation_est = np.random.normal(loc=rotation_1, scale=0.1 * abs(rotation_1), size=100)
#         print(type(sample_odom_est))
#         ####################################################################################
#         # Gaussian Noise Accumulation
#         idx_odom = 0
#         for sample_odom in zip(sample_odom_est.T):
                
#             idx_rotation = 0
#             for rotation in zip(rotation_est):
#                 # print(rotation[0])
#                 # plt.plot(translation * np.cos(pose[2] + rotation[0]) + sample_odom[0][0], translation * np.sin(pose[2] + rotation[0]) + sample_odom[0][1], 'x', alpha=0.5)

#                 # For each noisy pose estimation, apply translation and sample Gaussian distriubtion.
#                 # Covariance of the distribution depends on the value of translation.
#                 # This is because longer the robot travels, its odometry will have more likelihood to receive noise.
#                 mean = [translation * np.cos(pose[2] + rotation[0]) + sample_odom[0][0], translation * np.sin(pose[2] + rotation[0]) + sample_odom[0][1]]
#                 cov = [[translation, 0], 
#                         [0, translation]]
#                 odom_est = np.random.multivariate_normal(mean, cov, 100).T

#                 # Accumulate all the odometry distribution values
#                 if idx_odom == 0:
#                     concat_array = odom_est
#                     idx_odom += 1
#                 else:
#                     concat_array = np.concatenate((concat_array, odom_est), axis=1)

#         # Since it is not possible to use all the values in odometry distribution, acquire a sample from the distribution
#         sample_idx_mask = np.random.randint(low=0, high=concat_array.shape[1], size=100)

#         sample_odom_est = concat_array[:, sample_idx_mask]
#         #####################################################################################

#         pose = [pose[0] + dx, pose[1] + dy, pose[2] + dtheta]
#         groundtruth_poses.append(pose)             
#         print('Pose : {}'.format(pose))

#         plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
#         plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')
#         plt.arrow(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 20 * np.cos(groundtruth_poses[iteration+1][2]), 20 * np.sin(groundtruth_poses[iteration+1][2]), width=1)

#         plt.arrow(groundtruth_poses[iteration][0], groundtruth_poses[iteration][1], 20 * np.cos(groundtruth_poses[iteration][2] + rotation_1), 20 * np.sin(groundtruth_poses[iteration][2] + rotation_1), width=1, color='red')

#         pose_est_x = np.mean(sample_odom_est[0])
#         pose_est_y = np.mean(sample_odom_est[1])
#         print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
#         plt.plot(pose_est_x, pose_est_y, 'o')

# plt.show()