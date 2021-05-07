import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

class grid_based_odom:

    def __init__(self, init_x=100, init_y=100, init_theta=0, translation_variance_scale=0.1, rotation_variance=0.1):

        self.grid = np.zeros((200, 200), dtype=np.float64)

        self.grid[init_x, init_y] = 1.0

        x_mesh, y_mesh = np.mgrid[0:200:1, 0:200:1]

        self.xy_grid_mesh = np.dstack((x_mesh, y_mesh))

        self.pose_groundtruth = np.array([init_x, init_y, init_theta])     # x [m], y [m], orientation [radian]
        self.pose_est = np.array([init_x, init_y, init_theta])             # x [m], y [m], orientation [radian]

        self.groundtruth_path = [np.array(self.pose_groundtruth)]
        self.pose_est_path = [np.array(self.pose_est)]

        self.translation_variance_scale = translation_variance_scale
        self.rotation_variance = rotation_variance

        print('Initial Current Pose : {}'.format(self.pose_groundtruth))
        print('Initial Pose Estimation : {}'.format(self.pose_est))
        print('--------------------------------------------------------')

    def min_max_normalize(self, input):

        normalized = (input-np.min(input)) / (np.max(input)-np.min(input))

        return normalized

    def update_probability_map(self, command=[0, 0 ,0]):

        dx = command[0]
        dy = command[1]
        dtheta = command[2]

        print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

        translation = np.sqrt(dx**2 + dy**2)                     # Calculate the length of translation
        rotation_1 = np.arctan2(dy, dx) - self.pose_est[2]       # Calculate the rotation angle for aligning the heading
        rotation_2 = self.pose_est[2] + dtheta - rotation_1      # Calculate the rotation angle for aligning final heading

        print('Translation : {}'.format(translation))
        print('rotation_1 : {}'.format(rotation_1))
        print('rotation_2 : {}'.format(rotation_2))

        rotation_est = np.random.normal(loc=rotation_1, scale=self.rotation_variance, size=100)

        self.calc_grid = np.zeros((200, 200), dtype=np.float64)
        for rotation in zip(rotation_est):

            pose_est_distribution = multivariate_normal([self.pose_groundtruth[0] + translation * np.cos(rotation[0]), self.pose_groundtruth[1] + translation * np.sin(rotation[0])], 
                                                        [[self.translation_variance_scale * translation, 0], 
                                                         [0, self.translation_variance_scale * translation]])

            pose_est_distribution_value = pose_est_distribution.pdf(self.xy_grid_mesh)

            self.calc_grid += pose_est_distribution_value

        pose_est_idx = np.unravel_index(np.argmax(self.calc_grid), shape=self.calc_grid.shape)

        self.pose_est[0] = pose_est_idx[0]
        self.pose_est[1] = pose_est_idx[1]

        self.pose_groundtruth[0] += dx
        self.pose_groundtruth[1] += dy

        self.grid += self.min_max_normalize(self.calc_grid)

        self.groundtruth_path.append(np.array(self.pose_groundtruth))
        self.pose_est_path.append(np.array(self.pose_est))

        print('--------------------------------------------------------')

if __name__ == '__main__':

    ### Command Sequence for Main Odometry Scenario ###
    main_sequence_commands = np.array([[5, 0, 0], [10, 0, 0], [10, 0, 0.785], [10, 0, 1.57], [0, 10, -0.785], [10, 0, 0], [10, 0 , -0.785], [0, -30, 1.57], 
                                       [15, -15, 0], [10, 0, 0], [10, -10, 0.785], [10, 0, 1.57], [0, 10, -0.785], [10, 0, 0], [10, 0 , -0.785], [0, -30, 0],
                                       [-10, -10, 0], [-10, -20, 0], [-10, -10, 1.57], [-30, 0, 0], [-30, 0, 0], [-30, 0, 0], [0, 20, 1.57], [0, 30, 0], [0, 30, 0], [0, 25, 0]])

    Translation_Rotation_Variance = np.array([[0.5, 0.5], [1.0, 0.5], [0.5, 1.0], [1.0, 1.0]])

    for T_R_variance in zip(Translation_Rotation_Variance):

        Odom_estmiator = grid_based_odom(init_x=50, init_y=150, init_theta=0, 
                                         translation_variance_scale=T_R_variance[0][0],
                                         rotation_variance=T_R_variance[0][1])

        for odom_command in zip(main_sequence_commands):

            Odom_estmiator.update_probability_map(command=odom_command[0])

        plt.title('Probabilistic Odometry Pose Estimation on Discrete Grid\n' +
                  '[Translation Variance Scale = {}, Rotation Variance = {}]'.format(Odom_estmiator.translation_variance_scale, Odom_estmiator.rotation_variance))
        plt.plot(np.array(Odom_estmiator.groundtruth_path)[:, 0], np.array(Odom_estmiator.groundtruth_path)[:, 1], '--o', color='red')
        plt.plot(np.array(Odom_estmiator.pose_est_path)[:, 0], np.array(Odom_estmiator.pose_est_path)[:, 1], '-o', color='blue')
        plt.imshow(Odom_estmiator.grid.T, origin='lower', cmap='gray_r')
        plt.show()