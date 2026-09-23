#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""Car Agent 层组件抽象基类。

定义 SkillManager / TaskEngine 两大接口，
使 CarAgent 可以通过依赖注入使用任意实现了这些接口的组件。
"""

from abc import ABCMeta, abstractmethod


class BaseSkillManager(object):
    """技能管理器接口。

    管理单辆小车的所有 ROS 发布器/订阅器，
    以及技能实例的创建、切换与生命周期。
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def switch_skill(self, action_type, task):
        """停止当前技能并切换到指定类型的技能。"""
        pass

    @abstractmethod
    def update_active_skill(self):
        """执行当前技能的 update() 逻辑。

        返回:
            RUNNING / SUCCESS / FAILED
        """
        pass

    @abstractmethod
    def stop_active_skill(self):
        """停止并清理当前技能。"""
        pass

    @abstractmethod
    def set_task_feedback(self, task_id, current_action,
                          task_status, mode, reason=""):
        """设置任务反馈状态（供 RobotState 发布使用）。"""
        pass


class BaseTaskEngine(object):
    """任务引擎接口。

    维护当前任务，驱动技能执行，监控任务超时。
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def accept_task(self, msg):
        """接收并处理新的 TaskCommand 消息。"""
        pass

    @abstractmethod
    def tick(self):
        """主循环步进：检查超时、驱动技能 update、更新反馈状态。"""
        pass
