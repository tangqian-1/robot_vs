#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from skills.base_skill import BaseSkill, RUNNING, SUCCESS, FAILED


class GoToSkill(BaseSkill):
    """通过 move_base_simple/goal 导航到指定 (x, y) 目标。

    当 move_base 返回 SUCCEEDED（status == 3）时结束为 SUCCESS。
    当 move_base 返回失败状态时，尝试调整目标点并重试，最多重试3次。
    """

    # move_base 的 GoalStatus 状态码
    _STATUS_SUCCEEDED = 3
    _STATUS_FAILED = {4, 5, 8, 9}  # ABORTED、REJECTED、LOST 等

    def __init__(self, skill_manager, frame_id="map"):
        super(GoToSkill, self).__init__(skill_manager)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.frame_id = str(frame_id)
        self.retry_count = 0
        self.max_retries = 3
        self.occupied_threshold = int(rospy.get_param("~goto_occupied_threshold", 65))
        self.unknown_as_obstacle = bool(rospy.get_param("~goto_unknown_as_obstacle", True))
        self.search_step_m = float(rospy.get_param("~goto_adjust_step_m", 0.20))
        self.max_search_radius_m = float(rospy.get_param("~goto_max_search_radius_m", 1.20))
        self.clearance_m = float(rospy.get_param("~goto_clearance_m", 0.25))

    def start(self, params=None):
        params = params or {}
        self.target_x = float(params.get("target_x", 0.0))
        self.target_y = float(params.get("target_y", 0.0))
        self.target_yaw = float(params.get("target_yaw", 0.0))
        self.retry_count = 0

        # 初次下发仅做合理化（边界裁剪、距离过近处理），不做“失败重试偏移”。
        self.target_x, self.target_y = self._adjust_target(
            self.target_x,
            self.target_y,
            retry_count=0,
        )

        self._publish_goal(self.target_x, self.target_y, self.target_yaw)
        self._status = RUNNING

        rospy.loginfo(
            "[%s] GoToSkill start: target=(%.2f, %.2f) yaw=%.2f",
            self.skill_manager.ns, self.target_x, self.target_y, self.target_yaw,
        )

    def update(self):
        nav_status = self.skill_manager.nav_status_code

        if nav_status == self._STATUS_SUCCEEDED:
            self._status = SUCCESS
        elif nav_status in self._STATUS_FAILED:
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                # 仅在失败后做重试偏移。
                self.target_x, self.target_y = self._adjust_target(
                    self.target_x,
                    self.target_y,
                    retry_count=self.retry_count,
                )
                self._publish_goal(self.target_x, self.target_y, self.target_yaw)
                self._status = RUNNING
                rospy.loginfo(
                    "[%s] GoToSkill retry %d: adjusted target=(%.2f, %.2f) yaw=%.2f",
                    self.skill_manager.ns, self.retry_count, self.target_x, self.target_y, self.target_yaw,
                )
            else:
                rospy.logwarn(
                    "[%s] GoToSkill failed after %d retries: move_base status=%d",
                    self.skill_manager.ns, self.max_retries, nav_status,
                )
                self._status = FAILED
        else:
            self._status = RUNNING

        return self._status

    def _publish_goal(self, target_x, target_y, target_yaw):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(target_x)
        goal.pose.position.y = float(target_y)
        goal.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(target_yaw))
        goal.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.skill_manager.reset_nav_status()
        self.skill_manager.publish_nav_goal(goal)

    def _clamp_to_map(self, target_x, target_y):
        """将目标点裁剪到地图边界内（若地图信息可用）。"""
        map_info = self.skill_manager.get_map_info()
        if map_info:
            resolution = float(map_info.get('resolution', 0.0) or 0.0)
            width = float(map_info.get('width', 0) or 0)
            height = float(map_info.get('height', 0) or 0)
            if resolution <= 0.0 or width <= 0.0 or height <= 0.0:
                return target_x, target_y

            epsilon = min(1e-4, max(1e-6, resolution * 0.1)) if resolution > 0.0 else 1e-6

            min_x = float(map_info.get('origin_x', 0.0))
            max_x = min_x + width * resolution - epsilon
            min_y = float(map_info.get('origin_y', 0.0))
            max_y = min_y + height * resolution - epsilon

            target_x = max(min_x, min(target_x, max_x))
            target_y = max(min_y, min(target_y, max_y))
        return target_x, target_y

    def _is_navigable(self, x, y):
        checker = getattr(self.skill_manager, "is_world_point_navigable", None)
        if not callable(checker):
            return True
        clearance = float(self.clearance_m or 0.0)
        if clearance <= 1e-6:
            return bool(
                checker(
                    x,
                    y,
                    occupied_threshold=self.occupied_threshold,
                    unknown_as_obstacle=self.unknown_as_obstacle,
                )
            )

        map_info = self.skill_manager.get_map_info() or {}
        resolution = float(map_info.get('resolution', 0.0) or 0.0)
        width = int(map_info.get('width', 0) or 0)
        height = int(map_info.get('height', 0) or 0)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return bool(
                checker(
                    x,
                    y,
                    occupied_threshold=self.occupied_threshold,
                    unknown_as_obstacle=self.unknown_as_obstacle,
                )
            )

        to_index = getattr(self.skill_manager, "world_to_map_index", None)
        get_cell = getattr(self.skill_manager, "get_map_cell_value", None)
        if not callable(to_index) or not callable(get_cell):
            return bool(
                checker(
                    x,
                    y,
                    occupied_threshold=self.occupied_threshold,
                    unknown_as_obstacle=self.unknown_as_obstacle,
                )
            )

        center = to_index(x, y)
        if center is None:
            return False

        radius_cells = int(math.ceil(clearance / resolution))
        clearance_sq = clearance * clearance + 1e-9

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                dist_sq = (dx * resolution) ** 2 + (dy * resolution) ** 2
                if dist_sq > clearance_sq:
                    continue
                ix = center[0] + dx
                iy = center[1] + dy
                if ix < 0 or iy < 0 or ix >= width or iy >= height:
                    return False
                val = get_cell(ix, iy)
                if val is None:
                    continue
                if val < 0:
                    if self.unknown_as_obstacle:
                        return False
                    continue
                if val >= int(self.occupied_threshold):
                    return False

        return True

    def _find_nearby_navigable(self, center_x, center_y, retry_count):
        map_info = self.skill_manager.get_map_info() or {}
        resolution = float(map_info.get('resolution', 0.05) or 0.05)

        step = max(float(self.search_step_m), resolution)
        base_radius = 0.0 if retry_count <= 0 else min(self.max_search_radius_m, 0.25 * retry_count)
        radius = max(0.0, base_radius)
        max_radius = max(self.max_search_radius_m, radius)

        while radius <= (max_radius + 1e-6):
            if radius <= 1e-6:
                candidates = [(center_x, center_y)]
            else:
                # 均匀采样 16 个方向，快速寻找可通行替代点。
                candidates = []
                for i in range(16):
                    angle = (2.0 * math.pi / 16.0) * i
                    cx = center_x + radius * math.cos(angle)
                    cy = center_y + radius * math.sin(angle)
                    candidates.append(self._clamp_to_map(cx, cy))

            for cand_x, cand_y in candidates:
                if self._is_navigable(cand_x, cand_y):
                    return cand_x, cand_y

            radius += step

        return None

    def _find_along_to_robot(self, target_x, target_y, robot_x, robot_y, max_backtrack_m):
        map_info = self.skill_manager.get_map_info() or {}
        resolution = float(map_info.get('resolution', 0.05) or 0.05)
        step = max(float(self.search_step_m), resolution)

        dx = robot_x - target_x
        dy = robot_y - target_y
        dist = math.sqrt(dx**2 + dy**2)
        if dist <= 1e-6:
            return None

        max_backtrack = min(float(max_backtrack_m), dist)
        ux = dx / dist
        uy = dy / dist

        travel = 0.0
        while travel <= (max_backtrack + 1e-6):
            cand_x = target_x + ux * travel
            cand_y = target_y + uy * travel
            cand_x, cand_y = self._clamp_to_map(cand_x, cand_y)
            if self._is_navigable(cand_x, cand_y):
                return cand_x, cand_y
            travel += step

        return None

    def _adjust_target(self, target_x, target_y, retry_count=0):
        """规整/重试目标点。

        retry_count == 0: 仅做基础规整（边界裁剪 + 近距离前推）。
        retry_count > 0: 在失败后扩大半径搜索可通行候选点。
        """
        target_x, target_y = self._clamp_to_map(target_x, target_y)

        pose = self.skill_manager.get_current_pose()
        if pose is None:
            candidate = self._find_nearby_navigable(target_x, target_y, retry_count)
            return candidate if candidate is not None else (target_x, target_y)

        rx = pose.position.x
        ry = pose.position.y
        dx = target_x - rx
        dy = target_y - ry
        dist = math.sqrt(dx**2 + dy**2)
        offset = 0.5  # 偏移距离，单位米

        if dist < 0.1:  # 目标点太靠近机器人当前位置，偏移到机器人前方
            q = pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            new_x = rx + offset * math.cos(yaw)
            new_y = ry + offset * math.sin(yaw)
            target_x, target_y = self._clamp_to_map(new_x, new_y)

        if self._is_navigable(target_x, target_y):
            return target_x, target_y

        line_candidate = self._find_along_to_robot(
            target_x,
            target_y,
            rx,
            ry,
            self.max_search_radius_m,
        )
        if line_candidate is not None:
            adj_x, adj_y = line_candidate
            rospy.logwarn(
                "[%s] GoToSkill adjusted target toward robot: (%.2f, %.2f) -> (%.2f, %.2f)",
                self.skill_manager.ns,
                target_x,
                target_y,
                adj_x,
                adj_y,
            )
            return adj_x, adj_y

        candidate = self._find_nearby_navigable(target_x, target_y, retry_count)
        if candidate is not None:
            adj_x, adj_y = candidate
            rospy.logwarn(
                "[%s] GoToSkill adjusted blocked target: (%.2f, %.2f) -> (%.2f, %.2f)",
                self.skill_manager.ns,
                target_x,
                target_y,
                adj_x,
                adj_y,
            )
            return adj_x, adj_y

        rospy.logwarn(
            "[%s] GoToSkill no free candidate near target=(%.2f, %.2f), keep clamped point",
            self.skill_manager.ns,
            target_x,
            target_y,
        )
        return target_x, target_y

    def stop(self):
        # 通过发布零速度进行取消；move_base 将进入空闲。
        self.skill_manager.publish_stop_velocity()
