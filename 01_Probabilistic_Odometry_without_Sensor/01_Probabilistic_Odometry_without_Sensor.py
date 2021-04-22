import numpy as np

import matplotlib.pyplot as plt
from numpy import random

sequence_01_commands = np.array([[1, 0, 0], [1, 0, 1.57], [0, 1, -1.57], [1, 0, 0], [1, 0 , -1.57], [0, -1, 1.57], [1, 0, 0]])

# sequence_01_commands = np.array([[1, 0, 0], [1, 0, 1.57], [0, 1, -1.57]])
# sequence_02_commands = np.array([[1, 0, 0], [-1, 0, 0], [1, 0, 0], [-1, 0, 0], [1, 0, 0], [-1, 0, 0], [1, 0, 0], [-1, 0, 0]])
sequence_03_commands = np.array([[0, 0, 0.174], [0, 0, 0.348], [0, 0, 0.174], [0, 0, 0.522], [0, 0, 0.696], [0, 0, 0.87],])


def probabilstic_odom_estimation(commands, translation_variance_scale=0.1, rotation_variance_scale=0.1):

    #######################
    ### Initial Setting ###
    #######################

    pose = [0, 0, 0]    # x [m], y [m], orientation [radian]
    groundtruth_poses = []
    groundtruth_poses.append(pose)

    odom_est_poses = []
    odom_est_poses.append(pose)

    print('Initial pose : {}'.format(pose))
    print('-----------------------------------------------------')

    plt.plot(pose[0], pose[1], 'o')
    plt.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]), width=0.01)

    ##########################################
    ### Iterative Odometry Pose Estimation ###
    ##########################################

    sample_odom_est = np.ndarray(shape=(2, 1),buffer=np.array([pose]))  # Initial setup for odometry estimation samples

    for iteration, (command) in enumerate(zip(commands)):

        print('Iteration : {}'.format(iteration))
        
        dx = command[0][0]
        dy = command[0][1]
        dtheta = command[0][2]
        
        print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

        translation = np.sqrt(dx**2 + dy**2)
        rotation_1 = np.arctan2(dy, dx) - pose[2]
        rotation_2 = dtheta - rotation_1
        print('Translation : {}'.format(translation))
        print('rotation_1 : {}'.format(rotation_1))
        print('rotation_2 : {}'.format(rotation_2))

        # Rotation Noise
        rotation_est = np.random.normal(loc=rotation_1, scale=rotation_variance_scale * abs(rotation_1), size=100)
        
        ####################################################################################
        # Gaussian Noise Accumulation
        idx_odom = 0
        for sample_odom in zip(sample_odom_est.T):
                
            idx_rotation = 0
            for rotation in zip(rotation_est):
                
                # For each noisy pose estimation, apply translation and sample Gaussian distriubtion.
                # Covariance of the distribution depends on the value of translation.
                # This is because longer the robot travels, its odometry will have more likelihood to receive noise.
                mean = [translation * np.cos(pose[2] + rotation[0]) + sample_odom[0][0], translation * np.sin(pose[2] + rotation[0]) + sample_odom[0][1]]
                cov = [[translation_variance_scale * translation, 0], 
                       [0, translation_variance_scale * translation]]
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
        plt.arrow(groundtruth_poses[iteration+1][0], groundtruth_poses[iteration+1][1], 0.05 * np.cos(groundtruth_poses[iteration+1][2]), 0.05 * np.sin(groundtruth_poses[iteration+1][2]), width=0.01)

        plt.arrow(groundtruth_poses[iteration][0], groundtruth_poses[iteration][1], 0.05 * np.cos(groundtruth_poses[iteration][2] + rotation_1), 0.05 * np.sin(groundtruth_poses[iteration][2] + rotation_1), width=0.01, color='red')

        pose_est_x = np.mean(sample_odom_est[0])
        pose_est_y = np.mean(sample_odom_est[1])
        pose_est_theta = np.mean(rotation_est)

        odom_est_poses.append([pose_est_x, pose_est_y, pose_est_theta])

        print('Pose Est : [{}, {}]'.format(pose_est_x, pose_est_y))
        plt.plot(pose_est_x, pose_est_y, 'o')

        print('-----------------------------------------------------')

    groundtruth_poses = np.array(groundtruth_poses)
    odom_est_poses = np.array(odom_est_poses)

    plt.plot(groundtruth_poses[:, 0], groundtruth_poses[:, 1], '--', color='black')
    plt.plot(odom_est_poses[:, 0], odom_est_poses[:, 1], '-', color='red')

    plt.show()

if __name__ == '__main__':

    probabilstic_odom_estimation(sequence_03_commands, translation_variance_scale=0.01, rotation_variance_scale=0.01)