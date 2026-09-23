#!/usr/bin/env python3
"""Prompt DTO utilities for compact LLM inputs."""

from __future__ import annotations

import json
from typing import Any, Dict, List, Mapping, Optional, Sequence


def compact_json(value: Any) -> str:
	return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def build_my_state(local_state: Mapping[str, Any], robot_id: str = "") -> Dict[str, Any]:
	local_map = _safe_mapping(local_state)
	state_map = _extract_state_map(local_map)

	resolved_id = _read_text(
		local_map.get("robot_id"),
		state_map.get("robot_ns"),
		local_map.get("robot_ns"),
		state_map.get("id"),
		local_map.get("id"),
		robot_id,
	)
	if not resolved_id:
		resolved_id = str(robot_id or "unknown")

	hp = _read_number(local_map, state_map, "hp", 100.0)
	ammo = _read_number(local_map, state_map, "ammo", 0.0)
	x = _read_position_component(state_map, local_map, "x")
	y = _read_position_component(state_map, local_map, "y")
	yaw = _read_number(local_map, state_map, "yaw", 0.0)

	current_action = _read_text(state_map.get("current_action"), local_map.get("current_action"))
	task_status = _read_text(state_map.get("task_status"), local_map.get("task_status"))

	return {
		"id": resolved_id,
		"hp": _round_int(hp, 0),
		"ammo": _round_int(ammo, 0),
		"pos": {
			"x": _round_pos(x),
			"y": _round_pos(y),
			"yaw": _round_pos(yaw),
		},
		"current_action": current_action,
		"task_status": task_status,
	}


def build_teammates(team_context: Mapping[str, Any], self_id: str = "") -> List[Dict[str, Any]]:
	ctx = _safe_mapping(team_context)
	friendly = ctx.get("friendly", {})
	if not isinstance(friendly, Mapping):
		return []

	out: List[Dict[str, Any]] = []
	for rid in sorted(friendly.keys()):
		if self_id and rid == self_id:
			continue
		entry = friendly.get(rid)
		state_map = _extract_state_map(entry)
		if not state_map:
			continue

		entry_map = entry if isinstance(entry, Mapping) else {}

		robot_id = _read_text(rid, state_map.get("robot_ns"), state_map.get("id"))
		if not robot_id:
			continue

		x = _read_position_component(state_map, {}, "x")
		y = _read_position_component(state_map, {}, "y")
		hp = _read_number(state_map, {}, "hp", 0.0)
		current_action = _read_text(
			state_map.get("current_action"),
			state_map.get("action"),
			entry_map.get("current_action"),
			entry_map.get("action"),
		)
		task_status = _read_text(
			state_map.get("task_status"),
			_get_nested(state_map, ("task", "status")),
			entry_map.get("task_status"),
			_get_nested(entry_map, ("task", "status")),
		)
		reason = _read_text(
			state_map.get("reason"),
			_get_nested(state_map, ("task", "reason")),
			entry_map.get("reason"),
			_get_nested(entry_map, ("task", "reason")),
		)
		reason = _truncate_text(reason, 160)

		out.append(
			{
				"id": robot_id,
				"x": _round_pos(x),
				"y": _round_pos(y),
				"hp": _round_int(hp, 0),
				"current_action": current_action,
				"task_status": task_status,
				"reason": reason,
			}
		)

	return out


def build_enemies_in_sight(team_context: Mapping[str, Any]) -> List[Dict[str, Any]]:
	ctx = _safe_mapping(team_context)
	enemy = ctx.get("enemy", {})
	if not isinstance(enemy, Mapping):
		return []

	state = enemy.get("state", {})
	if not isinstance(state, Mapping):
		return []

	items: List[Any] = []
	visible = state.get("visible_enemies")
	if isinstance(visible, list):
		items = visible
	else:
		enemies = state.get("enemies")
		if isinstance(enemies, list):
			items = enemies
		elif "x" in state and "y" in state:
			items = [state]

	out: List[Dict[str, Any]] = []
	for idx, item in enumerate(items):
		if not isinstance(item, Mapping):
			continue
		if item.get("visible") is False:
			continue

		enemy_id = _read_text(item.get("id"), item.get("robot_ns"), item.get("name"))
		if not enemy_id:
			enemy_id = "enemy_{}".format(idx + 1)

		x = _read_position_component(item, {}, "x")
		y = _read_position_component(item, {}, "y")
		hp = _read_number(item, {}, "hp", 0.0)

		out.append(
			{
				"id": enemy_id,
				"x": _round_pos(x),
				"y": _round_pos(y),
				"hp": _round_int(hp, 0),
			}
		)

	out.sort(key=lambda item: item.get("id", ""))
	return out


def build_team_context_dto(
	team_context: Mapping[str, Any],
	self_id: str = "",
	teammates: Optional[Sequence[Mapping[str, Any]]] = None,
	enemies_in_sight: Optional[Sequence[Mapping[str, Any]]] = None,
) -> Dict[str, Any]:
	ctx = _safe_mapping(team_context)
	team_color = _read_text(ctx.get("team_color"))
	my_cars_raw = ctx.get("my_cars", [])
	my_cars: List[str] = []
	if isinstance(my_cars_raw, list):
		for item in my_cars_raw:
			text = _read_text(item)
			if text:
				my_cars.append(text)

	teammates_list = list(teammates) if teammates is not None else build_teammates(ctx, self_id=self_id)
	enemies_list = list(enemies_in_sight) if enemies_in_sight is not None else build_enemies_in_sight(ctx)

	return {
		"team_color": team_color,
		"my_cars": my_cars,
		"teammates": [dict(item) for item in teammates_list if isinstance(item, Mapping)],
		"enemies_in_sight": [dict(item) for item in enemies_list if isinstance(item, Mapping)],
	}


def _safe_mapping(value: Any) -> Dict[str, Any]:
	if isinstance(value, Mapping):
		return dict(value)
	return {}


def _extract_state_map(entry: Any) -> Dict[str, Any]:
	if isinstance(entry, Mapping):
		state = entry.get("state")
		if isinstance(state, Mapping):
			return dict(state)
		return dict(entry)
	return {}


def _get_nested(value: Any, keys: Sequence[str]) -> Optional[Any]:
	cursor = value
	for key in keys:
		if not isinstance(cursor, Mapping):
			return None
		cursor = cursor.get(key)
	return cursor


def _first_not_none(*values: Any) -> Optional[Any]:
	for value in values:
		if value is not None:
			return value
	return None


def _read_position_component(state_map: Mapping[str, Any], fallback_map: Mapping[str, Any], key: str) -> Any:
	primary = _first_not_none(
		_get_nested(state_map, ("pose", "position", key)),
		_get_nested(state_map, ("position", key)),
		_get_nested(state_map, ("pos", key)),
		state_map.get(key),
	)
	if primary is not None:
		return primary

	return _first_not_none(
		_get_nested(fallback_map, ("pose", "position", key)),
		_get_nested(fallback_map, ("position", key)),
		_get_nested(fallback_map, ("pos", key)),
		fallback_map.get(key),
	)


def _read_number(local_map: Mapping[str, Any], state_map: Mapping[str, Any], key: str, default: float) -> float:
	value = local_map.get(key)
	if value is None:
		value = state_map.get(key)
	try:
		return float(value)
	except (TypeError, ValueError):
		return float(default)


def _read_text(*values: Any) -> str:
	for value in values:
		text = str(value or "").strip()
		if text:
			return text
	return ""


def _truncate_text(text: str, max_chars: int = 160) -> str:
	value = str(text or "")
	if len(value) <= max_chars:
		return value
	if max_chars <= 3:
		return value[:max_chars]
	return value[: max_chars - 3] + "..."


def _round_pos(value: Any, digits: int = 2) -> float:
	try:
		return round(float(value), digits)
	except (TypeError, ValueError):
		return 0.0


def _round_int(value: Any, default: int = 0) -> int:
	try:
		return int(round(float(value)))
	except (TypeError, ValueError):
		return int(default)


__all__ = [
	"build_enemies_in_sight",
	"build_my_state",
	"build_teammates",
	"build_team_context_dto",
	"compact_json",
]
