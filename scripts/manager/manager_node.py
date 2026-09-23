#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import json
import os

import rospy
from robot_vs.msg import GameState
from std_msgs.msg import String

from interfaces import BaseObserver, BaseFormatter, BasePlanner, BaseDispatcher
from battle_state_formatter import BattleStateFormatter
from global_observer import GlobalObserver
from llm_client import LLMClient
from task_dispatcher import TaskDispatcher


try:
	text_type = unicode  # type: ignore[name-defined]
	binary_type = str
except NameError:
	text_type = str
	binary_type = bytes


class TeamManager(object):
	"""ROS1 团队管理主节点。

	核心循环：
	  1) 观测全局状态
	  2) 格式化规划输入
	  3) 调用 LLM 规划器生成任务
	  4) 分发任务
	"""

	def __init__(self, team_color="red", my_cars=None, loop_hz=0.2,
				 state_timeout_s=2.0, default_patrol_points=None,
				 enemy_topic="/referee/enemy_state",
				 llm_enabled=False,
				 llm_service_url="http://127.0.0.1:8001/plan",
				 llm_timeout_s=8.0):
		if my_cars is None:
			my_cars = []
		self.team_color = str(team_color)
		self.my_cars = list(my_cars)
		self.loop_hz = float(loop_hz)
		self.state_timeout_s = float(state_timeout_s)
		self.default_patrol_points = list(default_patrol_points) if default_patrol_points else []
		self.enemy_topic = str(enemy_topic)
		self.llm_enabled = bool(llm_enabled)
		self.llm_service_url = str(llm_service_url)
		self.llm_timeout_s = float(llm_timeout_s)

		self.observer = GlobalObserver(
			my_cars=self.my_cars,
			state_timeout=self.state_timeout_s,
			enemy_topic=self.enemy_topic,
		)
		self.formatter = BattleStateFormatter()
		self.llm_client = LLMClient(
			patrol_points=(self.default_patrol_points or None),
			use_llm=self.llm_enabled,
			llm_service_url=self.llm_service_url,
			llm_timeout_s=self.llm_timeout_s,
		)
		self.dispatcher = TaskDispatcher(
			my_cars=self.my_cars,
		)

		# ====== 比赛状态同步 ======
		self._game_status = "IDLE"
		self._game_state_sub = rospy.Subscriber(
			"/game/state", GameState, self._on_game_state, queue_size=10
		)

		# ====== 叙事事件（Manager 发到 /game/narrative，Referee 汇总写入文件）======
		self._narrative_pub = rospy.Publisher("/game/narrative", String, queue_size=100)

		rospy.loginfo(
			"TeamManager initialized: team_color=%s my_cars=%s loop_hz=%.3f state_timeout_s=%.2f enemy_topic=%s patrol_points=%s llm_enabled=%s llm_service_url=%s llm_timeout_s=%.2f",
			self.team_color,
			self.my_cars,
			self.loop_hz,
			self.state_timeout_s,
			self.enemy_topic,
			self.default_patrol_points,
			self.llm_enabled,
			self.llm_service_url,
			self.llm_timeout_s,
		)

	@classmethod
	def from_ros_params(cls):
		team_color = rospy.get_param("~team_color", "red")
		my_cars = rospy.get_param("~my_cars", [])
		loop_hz = rospy.get_param("~loop_hz", 0.2)
		state_timeout_s = rospy.get_param("~state_timeout_s", 2.0)
		default_patrol_points = rospy.get_param("~default_patrol_points", [])
		node_name = rospy.get_name().strip("/")
		default_enemy_topic = "/{}/enemy_state".format(node_name) if node_name else "/referee/enemy_state"
		enemy_topic = rospy.get_param("~enemy_topic", default_enemy_topic)
		llm_config = rospy.get_param("~llm", {})
		if not isinstance(llm_config, dict):
			llm_config = {}
		llm_enabled = rospy.get_param("~llm_enabled", llm_config.get("enabled", False))
		llm_service_url = rospy.get_param("~llm_service_url", llm_config.get("service_url", "http://127.0.0.1:8001/plan"))
		llm_timeout_s = rospy.get_param("~llm_timeout_s", llm_config.get("timeout_s", 8.0))

		cls._validate_params(team_color, my_cars, loop_hz, state_timeout_s, default_patrol_points, enemy_topic, llm_service_url, llm_timeout_s)
		return cls(
			team_color=team_color,
			my_cars=my_cars,
			loop_hz=loop_hz,
			state_timeout_s=state_timeout_s,
			default_patrol_points=default_patrol_points,
			enemy_topic=enemy_topic,
			llm_enabled=llm_enabled,
			llm_service_url=llm_service_url,
			llm_timeout_s=llm_timeout_s,
		)

	@staticmethod
	def _validate_params(team_color, my_cars, loop_hz, state_timeout_s, default_patrol_points, enemy_topic, llm_service_url, llm_timeout_s):
		if not isinstance(team_color, str):
			raise ValueError("~team_color must be a string")

		if not isinstance(my_cars, list):
			raise ValueError("~my_cars must be a list of strings")

		if not all(isinstance(car, str) and car for car in my_cars):
			raise ValueError("~my_cars must contain non-empty strings only")

		try:
			hz = float(loop_hz)
		except Exception:
			raise ValueError("~loop_hz must be a float")

		if hz <= 0.0:
			raise ValueError("~loop_hz must be > 0")

		try:
			timeout_s = float(state_timeout_s)
		except Exception:
			raise ValueError("~state_timeout_s must be a float")

		if timeout_s <= 0.0:
			raise ValueError("~state_timeout_s must be > 0")

		if not isinstance(default_patrol_points, list):
			raise ValueError("~default_patrol_points must be a list")

		if not isinstance(enemy_topic, str) or not enemy_topic:
			raise ValueError("~enemy_topic must be a non-empty string")

		if not isinstance(llm_service_url, str) or not llm_service_url:
			raise ValueError("~llm_service_url or ~llm.service_url must be a non-empty string")

		try:
			llm_timeout = float(llm_timeout_s)
		except Exception:
			raise ValueError("~llm_timeout_s or ~llm.timeout_s must be a float")

		if llm_timeout <= 0.0:
			raise ValueError("~llm_timeout_s or ~llm.timeout_s must be > 0")

	def build_fallback_tasks(self):
		fallback = {}
		for ns in self.my_cars:
			fallback[ns] = {
				"action": "STOP",
				"target": {"x": 0.0, "y": 0.0},
				"mode": 0,
				"reason": "fallback_on_exception in manager.py",
				"timeout": 2.0,
			}
		return fallback

	def _on_game_state(self, msg):
		self._game_status = str(msg.status)

	def _send_stop_to_all(self, reason="match_ended"):
		"""给所有小车发 STOP。"""
		for ns in self.my_cars:
			task = {
				"action": "STOP",
				"target": {"x": 0.0, "y": 0.0, "yaw": 0.0},
				"mode": 0,
				"reason": reason,
				"timeout": 2.0,
			}
			try:
				pub = self.dispatcher._ensure_publisher(ns)
				msg = self.dispatcher._build_task_msg(ns, task)
				pub.publish(msg)
			except Exception as exc:
				rospy.logwarn("stop failed for %s: %s", ns, exc)
		rospy.loginfo("[%s] STOP sent to %d robots (reason=%s)", self.team_color, len(self.my_cars), reason)

	def _publish_narrative(self, message):
		"""向 /game/narrative 发一条纯文本叙事。"""
		try:
			if isinstance(message, dict):
				text = json.dumps(message, ensure_ascii=True)
			else:
				text = self._to_text(message, u"")
			if text_type is not str:
				payload = text.encode("utf-8")
			else:
				payload = text
			self._narrative_pub.publish(String(payload))
		except Exception:
			pass

	def _to_text(self, value, default=u""):
		if value is None:
			value = default
		try:
			if isinstance(value, text_type):
				return value
			if isinstance(value, binary_type):
				return value.decode("utf-8", "replace")
			return text_type(value)
		except Exception:
			try:
				return text_type(default)
			except Exception:
				return u""

	def run_cycle(self):
		state = self.observer.get_battle_state()#状态字典
		prompt_input = self.formatter.build(state, self.team_color, self.my_cars)
		tasks = self.llm_client.plan_tasks(prompt_input)#任务字典
		self.dispatcher.dispatch(tasks)

		team_text = self._to_text(self.team_color, u"")

		# 1) 发布 Leader 战略理由（如果有）— 先于具体命令，提升可读性
		leader_order = getattr(self.llm_client, "last_leader_order", "")
		if leader_order:
			self._publish_narrative({
				"team": team_text,
				"event": "leader_order",
				"msg": u"[%s_leader] %s" % (team_text, self._to_text(leader_order, u"")),
			})

		# 2) 发布司令（Manager）的决策叙事
		actions_summary = []
		for ns, task in tasks.items():
			ns_text = self._to_text(ns, u"")
			action = self._to_text(task.get("action", "STOP"), u"STOP").upper()
			reason = self._to_text(task.get("reason", ""), u"")
			tgt = task.get("target", {})
			tgt_str = u"(%.2f,%.2f)" % (float(tgt.get("x", 0)), float(tgt.get("y", 0)))
			actions_summary.append(u"%s=%s%s" % (ns_text, action, tgt_str))
		self._publish_narrative(
			{
				"team": team_text,
				"event": "command",
				"msg": u"[%s_manager] order: %s" % (team_text, u", ".join(actions_summary)),
			},
		)

		# 3) 发布每条任务的叙事（含 reason），Referee 汇总写入队伍日志
		for ns, task in tasks.items():
			ns_text = self._to_text(ns, u"")
			action = self._to_text(task.get("action", "STOP"), u"STOP").upper()
			reason = self._to_text(task.get("reason", ""), u"")
			tgt = task.get("target", {})
			tgt_str = u"(%.2f, %.2f)" % (float(tgt.get("x", 0)), float(tgt.get("y", 0)))
			self._publish_narrative(
				{
					"team": team_text,
					"event": "command",
					"msg": u"[%s] %s %s - %s" % (ns_text, action, tgt_str, reason),
					"reason": reason,
				},
			)

		return tasks

	def run(self):
		rate = rospy.Rate(self.loop_hz)
		while not rospy.is_shutdown():
			if self._game_status == "FINISHED":
				# 比赛结束：持续发 STOP
				self._send_stop_to_all("match_ended")
				rate.sleep()
				continue

			if self._game_status == "IDLE":
				# 比赛未开始：空转等待
				rate.sleep()
				continue

			# _game_status == "PLAYING": 正常规划
			try:
				self.run_cycle()
			except Exception as exc:
				rospy.logwarn("TeamManager cycle failed: %s", exc)
				fallback_tasks = self.build_fallback_tasks()
				try:
					self.dispatcher.dispatch(copy.deepcopy(fallback_tasks))
				except Exception as dispatch_exc:
					rospy.logwarn("Fallback dispatch failed: %s", dispatch_exc)
			rate.sleep()


def main():
	rospy.init_node("team_manager")

	try:
		manager = TeamManager.from_ros_params()
	except Exception as exc:
		rospy.logwarn("TeamManager param/init error: %s", exc)
		# 当参数非法时使用保守默认值，确保节点保持可运行。
		node_name = rospy.get_name().strip("/")
		default_enemy_topic = "/{}/enemy_state".format(node_name) if node_name else "/referee/enemy_state"
		manager = TeamManager(team_color="red", my_cars=[], loop_hz=1, state_timeout_s=5.0, enemy_topic=default_enemy_topic)

	manager.run()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
