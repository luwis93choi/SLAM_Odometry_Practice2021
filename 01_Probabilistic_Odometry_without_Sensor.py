import numpy as np

import matplotlib.pyplot as plt
from numpy import random

discrete_grid = np.zeros((200, 200, 36), dtype=float)

groundtruth_poses = []

### Initial Setting ###

pose = [0, 0]
groundtruth_poses.append(pose)

print('Initial pose : {}'.format(pose))
plt.plot(pose[0], pose[1], 'x')

### Iterative Odometry Pose Estimation ###

# Control Commands

for iteration in range(5):

    print('Iteration : {}'.format(iteration))

    if iteration == 0:

        # Control Commands
        dx = random.randint(low=10, high=30, size=1)[0]
        dy = random.randint(low=10, high=30, size=1)[0]
        dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]

        pose = [pose[0] + dx, pose[1] + dy]
        groundtruth_poses.append([pose[0], pose[1]])

        translation = np.sqrt(np.sum(np.sqrt([dx, dy])))

        mean = [pose[0], pose[1]]
        cov = [[translation, 0], 
               [0, translation]]
        odom_est = np.random.multivariate_normal(mean, cov, 5000).T

        sample_idx_mask = np.random.randint(low=0, high=odom_est.shape[1], size=100)

        sample_odom_est = odom_est[:, sample_idx_mask]

        plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
        plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')

        pose_est_x = np.mean(sample_odom_est[0])
        pose_est_y = np.mean(sample_odom_est[1])
        print(pose_est_x)
        print(pose_est_y)
        plt.plot(pose_est_x, pose_est_y, 'o')

    else:
        dx = random.randint(low=10, high=30, size=1)[0]
        dy = random.randint(low=10, high=30, size=1)[0]
        dtheta = random.uniform(low=-3.14, high=3.14, size=1)[0]
        
        pose = [pose[0] + dx, pose[1] + dy]
        groundtruth_poses.append([pose[0], pose[1]])

        translation = np.sqrt(np.sum(np.sqrt([dx, dy])))

        mean = [pose_est_x + dx, pose_est_y + dy]
        cov = [[translation, 0], 
               [0, translation]]
        odom_est = np.random.multivariate_normal(mean, cov, 5000).T

        sample_idx_mask = np.random.randint(low=0, high=odom_est.shape[1], size=100)

        sample_odom_est = odom_est[:, sample_idx_mask]

        plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
        plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')

        pose_est_x = np.mean(sample_odom_est[0])
        pose_est_y = np.mean(sample_odom_est[1])
        print(pose_est_x)
        print(pose_est_y)
        plt.plot(pose_est_x, pose_est_y, 'o')

        # dx = random.randint(low=10, high=30, size=1)[0]
        # dy = random.randint(low=10, high=30, size=1)[0]
        # dx += 10
        # dy = 0
        # dtheta = 1

        # pose = [pose[0] + dx, pose[1] + dy]
        # groundtruth_poses.append([pose[0] + dx, pose[1] + dy])

        # translation = np.sqrt(np.sum(np.sqrt([dx, dy])))

        # idx = 0
        # for sample_odom in zip(sample_odom_est.T):

        #     mean = [sample_odom[0][0] + dx, sample_odom[0][1] + dy]
        #     cov = [[translation, 0], 
        #            [0, translation]]
        #     odom_est = np.random.multivariate_normal(mean, cov, 5000).T

        #     if idx == 0:
        #         concat_array = odom_est
        #         idx += 1

        #     else:
        #         concat_array = np.concatenate((concat_array, odom_est), axis=1)

        # print(concat_array.shape)

        # sample_idx_mask = np.random.randint(low=0, high=concat_array.shape[1], size=100)

        # sample_odom_est = concat_array[:, sample_idx_mask]

        # plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')
        # plt.plot(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 'o')

plt.axis('equal')
plt.show()