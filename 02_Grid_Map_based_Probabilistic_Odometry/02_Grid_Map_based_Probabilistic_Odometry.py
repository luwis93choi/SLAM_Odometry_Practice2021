import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt


class grid_based_odom:

    def __init__(self, x=100, y=100, theta=0):

        self.grid = np.zeros((200, 200), dtype=np.float64)

        self.grid[x, y] = 1.0

        x_mesh, y_mesh = np.mgrid[0:200:1, 0:200:1]

        self.xy_grid_mesh = np.dstack((x_mesh, y_mesh))

        self.pose = [x, y, theta]

        self.translation_variance_scale = 1.0
        self.rotation_variance_scale = 1.0

    def min_max_normalize(self, input):

        normalized = (input-np.min(input)) / (np.max(input)-np.min(input))

        return normalized

    def update_probability_map(self, command=[0, 0 ,0]):

        dx = command[0]
        dy = command[1]
        dtheta = command[2]

        print('[Control] dx : {}, dy : {}, dtheta : {}'.format(dx, dy, dtheta))

        translation = np.sqrt(dx**2 + dy**2)                 # Calculate the lenght of translation
        rotation_1 = np.arctan2(dy, dx) - self.pose[2]       # Calculate the rotation angle for aligning the heading
        rotation_2 = dtheta - rotation_1                     # Calculate the rotation angle for aligning final heading

        print('Translation : {}'.format(translation))
        print('rotation_1 : {}'.format(rotation_1))
        print('rotation_2 : {}'.format(rotation_2))

        # Rotation with Gaussian Noise
        rotation_est = np.random.normal(loc=rotation_1, scale=self.rotation_variance_scale * abs(rotation_1), size=100)
        
        for rotation in zip(rotation_est):

            pose_est_distribution = multivariate_normal([self.pose[0] + translation * np.cos(self.pose[2] + rotation[0]), self.pose[1] + translation * np.sin(self.pose[2] + rotation[0])], 
                                                        [[self.translation_variance_scale * translation, 0], 
                                                         [0, self.translation_variance_scale * translation]])

            pose_est_distribution = self.min_max_normalize(pose_est_distribution.pdf(self.xy_grid_mesh))

            self.grid += pose_est_distribution

        self.grid = self.min_max_normalize(self.grid)

        plt.imshow(self.grid)
        plt.show()


if __name__ == '__main__':

    Odom_estmiator = grid_based_odom()

    Odom_estmiator.update_probability_map(command=[10, 10, 5])