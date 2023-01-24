
import matplotlib.pyplot as plt
import roboticstoolbox as rtb

def plot_joint_trajectories(a1_list, a2_list, step=80):
    trajectory_solution = rtb.tools.trajectory.jtraj(a1_list, a2_list, step)
    q_array = trajectory_solution.q
    qd_array = trajectory_solution.qd
    qdd_array = trajectory_solution.qdd
    
    # 5th Order
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(16,10))
    fig.suptitle('Joint Trajectory Position, Velocity and Accelerations - Over Step')
    for i in range(6):
        q_data_joint = [q[i] for q in q_array]
        qd_data_joint = [qd[i] for qd in qd_array]
        qdd_data_joint = [qdd[i] for qdd in qdd_array]
        x_axes = range(step)
        if i<3:
            axes[0,i].plot(x_axes, q_data_joint, label="position")
            axes[0,i].plot(x_axes, qd_data_joint, label="velocity")
            axes[0,i].plot(x_axes, qdd_data_joint, label="acceleration")
            axes[0,i].set_title("Joint {}".format(str(i)))
            axes[0,i].set_xlabel("Step")
            axes[0,i].legend()
        else:
            axes[1,i-3].plot(x_axes, q_data_joint, label="position")
            axes[1,i-3].plot(x_axes, qd_data_joint, label="velocity")
            axes[1,i-3].plot(x_axes, qdd_data_joint, label="acceleration")
            axes[1,i-3].set_title("Joint {}".format(str(i)))
            axes[1,i-3].set_xlabel("Step")
            axes[1,i-3].legend()
    
    # quintic
    trajectory_solution = rtb.tools.trajectory.quintic(a1_list, a2_list, step)
    q_array = trajectory_solution.q
    qd_array = trajectory_solution.qd
    qdd_array = trajectory_solution.qdd
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(16,10))
    fig.suptitle('Joint Trajectory Position, Velocity and Accelerations - Over Time')
    for i in range(6):
        q_data_joint = [q[i] for q in q_array]
        qd_data_joint = [qd[i] for qd in qd_array]
        qdd_data_joint = [qdd[i] for qdd in qdd_array]
        x_axes = range(step)
        if i<3:
            axes[0,i].plot(x_axes, q_data_joint, label="position")
            axes[0,i].plot(x_axes, qd_data_joint, label="velocity")
            axes[0,i].plot(x_axes, qdd_data_joint, label="acceleration")
            axes[0,i].set_title("Joint {}".format(str(i)))
            axes[0,i].set_xlabel("Step")
            axes[0,i].legend()
        else:
            axes[1,i-3].plot(x_axes, q_data_joint, label="position")
            axes[1,i-3].plot(x_axes, qd_data_joint, label="velocity")
            axes[1,i-3].plot(x_axes, qdd_data_joint, label="acceleration")
            axes[1,i-3].set_title("Joint {}".format(str(i)))
            axes[1,i-3].set_xlabel("Step")
            axes[1,i-3].legend()
    
    fig.tight_layout()
    plt.show()
    
plot_joint_trajectories([0,0,0,0,0,0], [0.2, 0.4, 0.6, 0.8, 1.0, 1.2], 100)