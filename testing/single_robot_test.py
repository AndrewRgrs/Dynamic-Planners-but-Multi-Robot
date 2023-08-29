"""
This script simulates a single robot moving through the environment using one of the implemented planners
When running, click on plot to:
    left-click: Add circle obstacle
    right-click: Remove circle obstacle
"""
import sys
import matplotlib
import matplotlib.pyplot as plt
from functools import partial
import multirobot_helpers as mrh

sys.path.insert(1, '../')
sys.path.insert(1, '../algorithms')


if __name__ == '__main__':
    # SELECT: DRRT, DRRT_STAR, RRTX, or VEL_OBS:

    # algorithm = 'DRRT'
    # algorithm = 'DRRT_STAR'
    # algorithm = 'RRTX'
    algorithm = 'VEL_OBS'

    max_iterations = 10000
    # Parameters for sampling based planners:
    rrt_params = {
        'iter_max': max_iterations,
        'robot_radius': 0.5,
        'step_len': 5.0,
        'move_dist': 0.02,  # must be < 0.05 bc that's used in update_robot_position()
        'gamma_FOS': 5.0,
        'epsilon': 0.05,
        'bot_sample_rate': 0.10,
        'waypoint_sample_rate': 0.5,
        'starting_nodes': 500,
        'node_limit': 5000,  # for each robot. after this, new nodes only added if robot gets orphaned
    }
    # Velocity Obstacle Parameters
    vel_obs_params = {
        'iter_max': max_iterations,
        'robot_radius': 0.5,
        'move_dist': 0.01,  # how far it can move per timestep
        'obstacle_FOS': 1,  # Repelling force of static obstacles
        'robot_FOS': 2,  # Repelling force of other robots
        'obstacle_radius': 20  # how far away robot can see other obstacles
    }

    top_left = (4, 4)
    top_right = (48, 4)
    bottom_left = (4, 28)
    bottom_right = (48, 28)

    # Start and Goal positions
    r1_start = (top_left[0] + 1, top_left[1] + 1)
    r1_goal = (bottom_right[0] - 1, bottom_right[1] - 1)

    if algorithm == 'DRRT':
        from algorithms.drrt import DRRT
        r1 = DRRT(
            x_start=r1_start,
            x_goal=r1_goal,
            robot_radius=rrt_params['robot_radius'],
            step_len=rrt_params['step_len'],
            move_dist=rrt_params['move_dist'],
            bot_sample_rate=rrt_params['bot_sample_rate'],
            waypoint_sample_rate=rrt_params['waypoint_sample_rate'],
            starting_nodes=rrt_params['starting_nodes'],
            node_limit=rrt_params['node_limit'],
            multi_robot=False,
            plot_params={
                'robot': True,
                'goal': True,
                'tree': True,
                'path': True,
                'nodes': True,
                'robot_color': 'blue',
                'tree_color': 'lightblue',
                'path_color': 'blue',
            }
        )
    elif algorithm == 'DRRT_STAR':
        from algorithms.drrt_star import DRRTStar
        r1 = DRRTStar(
            x_start=r1_start,
            x_goal=r1_goal,
            robot_radius=rrt_params['robot_radius'],
            step_len=rrt_params['step_len'],
            move_dist=rrt_params['move_dist'],
            gamma_FOS=rrt_params['gamma_FOS'],
            bot_sample_rate=rrt_params['bot_sample_rate'],
            waypoint_sample_rate=rrt_params['waypoint_sample_rate'],
            starting_nodes=rrt_params['starting_nodes'],
            node_limit=rrt_params['node_limit'],
            multi_robot=False,
            plot_params={
                'robot': True,
                'goal': True,
                'tree': True,
                'path': True,
                'nodes': True,
                'robot_color': 'blue',
                'tree_color': 'lightblue',
                'path_color': 'blue',
            }
        )
    elif algorithm == 'RRTX':
        from algorithms.rrtx import RRTX

        r1 = RRTX(
            x_start=r1_start,
            x_goal=r1_goal,
            robot_radius=rrt_params['robot_radius'],
            step_len=rrt_params['step_len'],
            move_dist=rrt_params['move_dist'],
            gamma_FOS=rrt_params['gamma_FOS'],
            epsilon=rrt_params['epsilon'],
            bot_sample_rate=rrt_params['bot_sample_rate'],
            starting_nodes=rrt_params['starting_nodes'],
            node_limit=rrt_params['node_limit'],
            multi_robot=True,
            plot_params={
                'robot': True,
                'goal': True,
                'tree': True,
                'path': True,
                'nodes': True,
                'robot_color': 'blue',
                'tree_color': 'lightblue',
                'path_color': 'blue',
            }
        )
    elif algorithm == 'VEL_OBS':
        from algorithms.velocity_obstacle import Velocity_Obstacle
        r1 = Velocity_Obstacle(
            start=r1_start,
            goal=r1_goal,
            robot_radius=vel_obs_params['robot_radius'],
            move_dist=vel_obs_params['move_dist'],
            iter_max=vel_obs_params['iter_max'],
            obstacle_FOS=vel_obs_params['obstacle_FOS'],
            robot_FOS=vel_obs_params['robot_FOS'],
            obstacle_radius=vel_obs_params['obstacle_radius'],
            plot_params={
                'robot': True,
                'goal': True,
                'tree': False,
                'path': False,
                'nodes': False,
                'robot_color': 'blue',
            }
        )

    # plotting stuff
    matplotlib.use('TkAgg')
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle(f'Planner: {algorithm}')
    ax.set_xlim(r1.env.x_range[0], r1.env.x_range[1] + 1)
    ax.set_ylim(r1.env.y_range[0], r1.env.y_range[1] + 1)
    ax.set_aspect('equal', adjustable='box')

    canvas = ax.figure.canvas
    canvas.draw()
    # mrh.env_plot(ax, r1)
    plt.pause(0.1)

    bg = canvas.copy_from_bbox(ax.bbox)
    canvas.blit(ax.bbox)

    # event handling (Left click: add obstacle, right click: remove obstacle
    canvas.mpl_connect('button_press_event', partial(mrh.update_click_obstacles, robots=[r1]))
    for i in range(max_iterations):
        r1.step()
        # update plot
        if r1.started and i % 30 == 0:
            canvas.restore_region(bg)
            mrh.env_plot(ax, r1)  # pass in any robot, they all know the environment
            mrh.single_bot_plot(ax, r1, sample_based=True if algorithm != 'VEL_OBS' else False)
            canvas.blit(ax.bbox)
            canvas.flush_events()