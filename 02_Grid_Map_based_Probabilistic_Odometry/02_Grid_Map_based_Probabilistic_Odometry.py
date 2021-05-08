import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

#############################################
### Probabilistic Odometry Pose Estimator ###
#############################################
class grid_based_odom:

    ##############################################
    ### Odometry Pose Estimator Initialization ###
    ##############################################
    def __init__(self, init_x=100, init_y=100, init_theta=0, translation_variance_scale=0.1, rotation_variance=0.1):

        self.grid = np.zeros((200, 200), dtype=np.float64)      # Probabilistic map grid for display

        self.grid[init_x, init_y] = 1.0                 # Initialize the initial position's probability as 1.0

        # x, y grid mesh initialization for Gaussian probability distribution of pose estimation
        x_mesh, y_mesh = np.mgrid[0:200:1, 0:200:1]     
        self.xy_grid_mesh = np.dstack((x_mesh, y_mesh))

        self.pose_groundtruth = np.array([init_x, init_y, init_theta])     # x [m], y [m], orientation [radian]
        self.pose_est = np.array([init_x, init_y, init_theta])             # x [m], y [m], orientation [radian]

        self.groundtruth_path = [np.array(self.pose_groundtruth)]          # Groundtruth path trajectory
        self.pose_est_path = [np.array(self.pose_est)]                     # Estimated path trajectory

        self.translation_variance_scale = translation_variance_scale       # Translation variance scale for translation noise
        self.rotation_variance = rotation_variance                         # Rotation variance for rotation noise

        print('Initial Current Pose : {}'.format(self.pose_groundtruth))
        print('Initial Pose Estimation : {}'.format(self.pose_est))
        print('--------------------------------------------------------')

    ### Min Max Normalization for Pose Probability Accumulation ###
    def min_max_normalize(self, input):

        normalized = (input-np.min(input)) / (np.max(input)-np.min(input))

        return normalized

    ##################################################################
    ### Probability Map Update by Accumulating Pose Estimation PDF ###
    ##################################################################
    def update_probability_map(self, command=[0, 0 ,0]):

        dx = command[0]         # Command for X-axis translation
        dy = command[1]         # Command for Y-axis translation
        dtheta = command[2]     # Command for rotation change

        print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

        translation = np.sqrt(dx**2 + dy**2)        # Calculate the length of translation
        rotation_1 = np.arctan2(dy, dx)             # Calculate the rotation angle for aligning the heading
        rotation_2 = self.pose_est[2] + dtheta      # Calculate the rotation angle for aligning final heading

        print('Translation : {}'.format(translation))
        print('rotation_1 : {}'.format(rotation_1))
        print('rotation_2 : {}'.format(rotation_2))

        # Align the heading towards the next position
        # Rotation towards the next position with Gaussian Noise / Variance is propotional to rotation variance setting
        rotation_est = np.random.normal(loc=rotation_1, scale=self.rotation_variance, size=100)

        ##########################################
        ### Iterative Odometry Pose Estimation ###
        ##########################################
        self.calc_grid = np.zeros((200, 200), dtype=np.float64)     # Probability map grid for current pose estimation accumulation
        for rotation in zip(rotation_est):          # Iterate over all the estimated rotation sequence with Gaussian noise
                                                    # Bigger rotation angle and variance, wider the overall pose estimation distribution

            # In each rotation sequence, produce Gaussian distriubtion of pose estimation.
            # Covariance of the pose estimation distribution depends on the value of translation.
            # This is because longer the robot travels, its odometry will have more likelihood to receive noise.
            pose_est_distribution = multivariate_normal([self.pose_groundtruth[0] + translation * np.cos(rotation[0]), self.pose_groundtruth[1] + translation * np.sin(rotation[0])], 
                                                        [[self.translation_variance_scale * translation, 0], 
                                                         [0, self.translation_variance_scale * translation]])

            pose_est_distribution_value = pose_est_distribution.pdf(self.xy_grid_mesh)  # Produce PDF of pose estimation distribution in current discrete probability map grid

            self.calc_grid += pose_est_distribution_value       # Accumulate all the PDF of pose estimation distribution

        # Find the estimated pose coordinate from mean of top 10 estimation probabilty values
        pose_est_idx = np.unravel_index(np.argsort(np.ravel(self.calc_grid))[-10:], shape=self.calc_grid.shape)
        
        # Update current pose estimation
        self.pose_est[0] = np.mean(pose_est_idx[0])
        self.pose_est[1] = np.mean(pose_est_idx[1])
        self.pose_est[2] = np.random.normal(loc=self.pose_est[2] + dtheta, scale=self.rotation_variance, size=1)[0]

        # Update groundtruth pose
        self.pose_groundtruth[0] += dx
        self.pose_groundtruth[1] += dy
        self.pose_groundtruth[2] += dtheta

        # Update discrete probabilty map with normalized value for display
        self.grid += self.min_max_normalize(self.calc_grid)

        # Accumulate current pose estimation and grountruth for path plotting
        self.groundtruth_path.append(np.array(self.pose_groundtruth))
        self.pose_est_path.append(np.array(self.pose_est))

        print('--------------------------------------------------------')

if __name__ == '__main__':

    ### Command Sequence for Main Odometry Scenario ###
    main_sequence_commands = np.array([[5, 0, 0], [10, 0, 0], [10, 0, 0.785], [10, 0, 1.57], [0, 10, -0.785], [10, 0, 0], [10, 0 , -0.785], [0, -30, 1.57], 
                                       [15, -15, 0], [10, 0, 0], [10, -10, 0.785], [10, 0, 1.57], [0, 10, -0.785], [10, 0, 0], [10, 0 , -0.785], [0, -30, 0],
                                       [-10, -10, 0], [-10, -20, 0], [-10, -10, 1.57], [-30, 0, 0], [-30, 0, 0], [-30, 0, 0], [0, 20, 1.57], [0, 30, 0], [0, 30, 0], [0, 25, 0]])

    ### Translation and Rotation Variance Scenario ###
    Translation_Rotation_Variance = np.array([[0.5, 0.5], [1.0, 0.5], [0.5, 1.0], [1.0, 1.0]])

    ### Iterate over each Translation and Rotation Variance Setting ###
    for T_R_variance in zip(Translation_Rotation_Variance):

        # Initialize discrete probabilty map-based odometry pose estimator with current translation and rotation variance setting
        Odom_estmiator = grid_based_odom(init_x=50, init_y=150, init_theta=0.785, 
                                         translation_variance_scale=T_R_variance[0][0],
                                         rotation_variance=T_R_variance[0][1])

        # Run through odometry scenario and Update the discrete probabilty map
        for odom_command in zip(main_sequence_commands):

            Odom_estmiator.update_probability_map(command=odom_command[0])

        print('Groundtruth Path : {}'.format(Odom_estmiator.groundtruth_path))
        print('Estimated Pose Path : {}'.format(Odom_estmiator.pose_est_path))

        # Plot groundtruth path trajectory and estimated path trajectory with pose estimation probability distribution
        plt.title('Probabilistic Odometry Pose Estimation on Discrete Grid\n' +
                  '[Translation Variance Scale = {}, Rotation Variance = {}]'.format(Odom_estmiator.translation_variance_scale, Odom_estmiator.rotation_variance))
        plt.plot(np.array(Odom_estmiator.groundtruth_path)[:, 0], np.array(Odom_estmiator.groundtruth_path)[:, 1], '--o', color='red', label='Groundtruth Path Trajectory')
        plt.plot(np.array(Odom_estmiator.pose_est_path)[:, 0], np.array(Odom_estmiator.pose_est_path)[:, 1], '-o', color='blue', label='Estimated Pose Path Trajectory')
        plt.legend(['Groundtruth Path Trajectory', 'Estimated Pose Path Trajectory'])
        plt.imshow(Odom_estmiator.grid.T, origin='lower', cmap='gray_r')
        plt.show()