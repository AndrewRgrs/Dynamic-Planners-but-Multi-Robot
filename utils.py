import math
import numpy as np
import os
import sys
from shapely.geometry import LineString
from shapely.geometry import Point

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

import env
# from rrtx import Node


class Utils:
    def __init__(self):
        self.env = env.Env()

        self.delta = 0.5
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            # shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            # dist_obs = self.get_dist(start, shot)
            dist_obs = math.hypot(start.x - (o[0] + t1 * d[0]), 
                                  start.y - (o[1] + t1 * d[1]))
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, ln1, ln2, a, r):

        # fast preliminary check
        unsafe_dist = r + math.hypot(ln1[0] - ln2[0], ln1[1] - ln2[1]) + self.delta
        if math.hypot(a[0] - ln2[0], a[1] - ln2[1]) > unsafe_dist or \
                math.hypot(a[0] - ln1[0], a[1] - ln1[1]) > unsafe_dist:
            return False

        p = Point(*a)
        c = p.buffer(r).boundary
        l = LineString([ln1, ln2])
        return c.intersects(l)

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(start, end, [x, y], r):
                return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    @staticmethod
    def update_robot_position(pos, bot, speed, dt):
        if not bot.parent:
            return bot, pos
        dx = bot.parent.x - pos[0]
        dy = bot.parent.y - pos[1]

        dist = speed * dt
        angle = math.atan2(dy, dx)

        new_pos = [pos[0] + dist * math.cos(angle), pos[1] + dist * math.sin(angle)]

        if math.hypot(dx, dy) > 0.05:
            return bot, new_pos
        else:
            return bot.parent, new_pos

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

    @staticmethod
    def get_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
