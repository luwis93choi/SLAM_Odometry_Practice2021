import numpy as np

import matplotlib.pyplot as plt

discrete_grid = np.zeros((200, 200, 36), dtype=float)

initial_pose = [0, 0]

dx = 10
dy = 5

mean = [initial_pose[0] + dx, initial_pose[1] + dy]
cov = [[1, 0], [0, 100]]
odom_est = np.random.multivariate_normal(mean, cov, 5000).T

sample_idx_mask = np.random.randint(low=0, high=odom_est.shape[1], size=500)
print(sample_idx_mask.shape)

sample_odom_est = odom_est[:, sample_idx_mask]
print(sample_odom_est.shape)

dx = 100
dy = 5

plt.plot(initial_pose[0], initial_pose[1], 'x')

plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')

for sample_odom in zip(sample_odom_est.T):

    mean = [sample_odom[0][0] + dx, sample_odom[0][1] + dy]
    cov = [[1, 0], [0, 100]]
    odom_est = np.random.multivariate_normal(mean, cov, 5000).T

    print(odom_est.shape)

    sample_idx_mask = np.random.randint(low=0, high=odom_est.shape[1], size=500)

    sample_odom_est = odom_est[:, sample_idx_mask]

    plt.plot(sample_odom_est[0], sample_odom_est[1], 'x')

plt.axis('equal')
plt.show()