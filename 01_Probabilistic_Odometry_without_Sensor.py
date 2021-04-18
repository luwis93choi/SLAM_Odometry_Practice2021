import numpy as np

import matplotlib.pyplot as plt
from numpy import random

discrete_grid = np.zeros((200, 200, 36), dtype=float)

groundtruth_poses = []

#######################
### Initial Setting ###
#######################

pose = [0, 0]
groundtruth_poses.append(pose)

print('Initial pose : {}'.format(pose))
plt.plot(pose[0], pose[1], 'x')

##########################################
### Iterative Odometry Pose Estimation ###
##########################################
for iteration in range(30):

    print('Iteration : {}'.format(iteration))

    # At the first iteration, acquire the odometry distribution from initial pose
    if iteration == 0:

        # Control Commands
        dx = random.randint(low=10, high=220, size=1)[0]
        dy = random.randint(low=10, high=110, size=1)[0]
        # dy = 0
        dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]

        pose = [pose[0] + dx, pose[1] + dy]
        groundtruth_poses.append([pose[0], pose[1]])

        translation = np.sqrt(dx**2 + dy**2)
        print('Translation : {}'.format(translation))

        mean = [pose[0], pose[1]]
        cov = [[translation, 0], 
               [0, translation]]
        odom_est = np.random.multivariate_normal(mean, cov, 500).T

        sample_idx_mask = np.random.randint(low=0, high=odom_est.shape[1], size=500)

        sample_odom_est = odom_est

        plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
        plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')

        pose_est_x = np.mean(sample_odom_est[0])
        pose_est_y = np.mean(sample_odom_est[1])
        print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
        plt.plot(pose_est_x, pose_est_y, 'o')

    # Iteratively acquire odometry distribution from each odometry estimation and accumulate it
    else:
        dx = random.randint(low=10, high=220, size=1)[0]
        dy = random.randint(low=10, high=110, size=1)[0]
        # dx += 100
        # dy = 0
        dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]
        
        pose = [pose[0] + dx, pose[1] + dy]
        groundtruth_poses.append([pose[0], pose[1]])

        translation = np.sqrt(dx**2 + dy**2)
        print('Translation : {}'.format(translation))

        ####################################################################################
        # Gaussian Noise Accumulation
        idx = 0
        for sample_odom in zip(sample_odom_est.T):

            # For each noisy pose estimation, apply translation and sample Gaussian distriubtion.
            # Covariance of the distribution depends on the value of translation.
            # This is because longer the robot travels, its odometry will have more likelihood to receive noise.
            mean = [sample_odom[0][0] + dx, sample_odom[0][1] + dy]
            cov = [[translation, 0], 
                   [0, translation]]
            odom_est = np.random.multivariate_normal(mean, cov, 100).T

            # Accumulate all the odometry distribution values
            if idx == 0:
                concat_array = odom_est
                idx += 1
            else:
                concat_array = np.concatenate((concat_array, odom_est), axis=1)

        # Since it is not possible to use all the values in odometry distribution, acquire a sample from the distribution
        sample_idx_mask = np.random.randint(low=0, high=concat_array.shape[1], size=500)

        sample_odom_est = concat_array[:, sample_idx_mask]
        #####################################################################################

        plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
        plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')

        pose_est_x = np.mean(sample_odom_est[0])
        pose_est_y = np.mean(sample_odom_est[1])
        print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
        plt.plot(pose_est_x, pose_est_y, 'o')

plt.show()