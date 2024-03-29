import matplotlib
import matplotlib.pyplot as plt
from functools import partial

import sys

sys.path.insert(1, '../')
sys.path.insert(1, '../algorithms')


from algorithms.rrtx import RRTX, Node
import multirobot_helpers as mrh

if __name__ == '__main__':
    plot_tree = [True,False]*2
    rrt_params = {
        'iter_max': 10_000,
        'robot_radius': 0.5,
        'step_len': 5.0,
        'move_dist': 0.02,  # must be < 0.05 bc that's used in update_robot_position()
        'gamma_FOS': 5.0,
        'epsilon': 0.05,
        'bot_sample_rate': 0.10,
        'starting_nodes': 500,
        'node_limit': 5000,  # for each robot. after this, new nodes only added if robot gets orphaned
    }

    top_left = (4, 4)
    top_right = (48, 4)
    bottom_left = (4, 28)
    bottom_right = (48, 28)

    r1_start = (top_left[0] + 1, top_left[1] + 1)
    r1_goal = (bottom_right[0] - 1, bottom_right[1] - 1)

    r2_start = (top_right[0] + 1, top_right[1] + 1)
    r2_goal = (bottom_left[0] - 1, bottom_left[1] - 1)

    r3_start = (bottom_left[0] + 1, bottom_left[1] + 1)
    r3_goal = (top_right[0] - 1, top_right[1] - 1)

    r4_start = (bottom_right[0] + 1, bottom_right[1] + 1)
    r4_goal = (top_left[0] - 1, top_left[1] - 1)

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
            'tree': plot_tree[0],
            'path': True,
            'nodes': False,
            'robot_color': 'blue',
            'tree_color': 'blue',
            'path_color': 'blue',
        }
    )

    r2 = RRTX(
        x_start=r2_start,
        x_goal=r2_goal,
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
            'tree': plot_tree[1],
            'path': True,
            'nodes': False,
            'robot_color': 'green',
            'tree_color': 'green',
            'path_color': 'green',
        }
    )

    r3 = RRTX(
        x_start=r3_start,
        x_goal=r3_goal,
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
            'tree': plot_tree[2],
            'path': True,
            'nodes': False,
            'robot_color': 'orange',
            'tree_color': 'orange',
            'path_color': 'orange',
        }
    )

    r4 = RRTX(
        x_start=r4_start,
        x_goal=r4_goal,
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
            'tree': plot_tree[3],
            'path': True,
            'nodes': False,
            'robot_color': 'brown',
            'tree_color': 'brown',
            'path_color': 'brown',
        }
    )

    robots = [r1, r2, r3, r4]

    for robot in robots:
        robot.set_other_robots([other for other in robots if other != robot])

    # plotting stuff
    matplotlib.use('TkAgg')
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle('RRTX')
    ax.set_xlim(r1.env.x_range[0], r1.env.x_range[1] + 1)
    ax.set_ylim(r1.env.y_range[0], r1.env.y_range[1] + 1)
    ax.set_aspect('equal', adjustable='box')
    canvas = ax.figure.canvas
    canvas.draw()
    # mrh.env_plot(ax, robots[0])

    plt.pause(0.1)

    bg = canvas.copy_from_bbox(ax.bbox)

    # plt.show(block=False)

    canvas.blit(ax.bbox)

    # event handling
    canvas.mpl_connect('button_press_event', partial(mrh.update_click_obstacles, robots=robots))
    for i in range(rrt_params['iter_max']):
        # RRTX step for each robot
        for robot in robots:
            robot.step()

        # update plot
        if robots[0].started and i % 30 == 0:
            canvas.restore_region(bg)
            mrh.env_plot(ax, robots[0])  # pass in any robot, they all know the environment
            for robot in robots:
                mrh.single_bot_plot(ax, robot)
            canvas.blit(ax.bbox)
            canvas.flush_events()

    print('\nRRTX complete!')
