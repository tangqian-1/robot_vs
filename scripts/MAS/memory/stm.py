#!/usr/bin/env python3
"""Short-term memory for recent battle-state snapshots."""

from __future__ import annotations

import asyncio
import copy
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class STMEntry:
	timestamp_s: float
	state: Dict[str, Any]
	source: str = "battle_state"
	note: str = ""


class ShortTermMemory:
	"""Recent-memory sliding window used by LeaderAgent."""

	def __init__(self, max_items: int = 12) -> None:
		if int(max_items) <= 0:
			raise ValueError("max_items must be > 0")
		self.max_items = int(max_items)
		self._entries: Deque[STMEntry] = deque(maxlen=self.max_items)
		self._lock = asyncio.Lock()

	async def append(
		self,
		state: Optional[Mapping[str, Any]],
		source: str = "battle_state",
		note: str = "",
		timestamp_s: Optional[float] = None,
	) -> STMEntry:
		snapshot = _safe_state_copy(state)
		entry = STMEntry(
			timestamp_s=float(timestamp_s) if timestamp_s is not None else time.time(),
			state=snapshot,
			source=str(source or "battle_state"),
			note=str(note or ""),
		)
		async with self._lock:
			self._entries.append(entry)
		return entry

	async def extend(
		self,
		states: Sequence[Mapping[str, Any]],
		source: str = "batch",
		timestamp_s: Optional[float] = None,
	) -> int:
		count = 0
		stamp = float(timestamp_s) if timestamp_s is not None else time.time()
		async with self._lock:
			for state in states:
				self._entries.append(
					STMEntry(
						timestamp_s=stamp,
						state=_safe_state_copy(state),
						source=str(source or "batch"),
						note="",
					)
				)
				count += 1
		return count

	async def latest(self) -> Optional[STMEntry]:
		async with self._lock:
			if not self._entries:
				return None
			return self._entries[-1]

	async def recent(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
		async with self._lock:
			snapshot = list(self._entries)

		if limit is not None:
			snapshot = snapshot[-max(0, int(limit)) :]

		return [
			{
				"timestamp_s": item.timestamp_s,
				"source": item.source,
				"note": item.note,
				"state": copy.deepcopy(item.state),
			}
			for item in snapshot
		]

	async def clear(self) -> None:
		async with self._lock:
			self._entries.clear()

	async def size(self) -> int:
		async with self._lock:
			return len(self._entries)

	async def summarize(self, max_lines: int = 8) -> str:
		"""Return concise semantic summary text for prompts."""
		async with self._lock:
			entries = list(self._entries)

		if not entries:
			return "No short-term memory available."

		first = entries[0].state
		last = entries[-1].state
		duration_s = max(0.0, entries[-1].timestamp_s - entries[0].timestamp_s)

		lines: List[str] = []
		lines.append("STM window: {} snapshots over {:.1f}s.".format(len(entries), duration_s))

		friendly_latest = _extract_friendly(last)
		enemy_latest = _extract_enemy(last)
		visible_enemy_count = _count_visible_enemies(enemy_latest)

		status_line = _build_friendly_status_line(friendly_latest, visible_enemy_count)
		if status_line:
			lines.append(status_line)

		movement_line = _build_movement_line(_extract_friendly(first), friendly_latest)
		if movement_line:
			lines.append(movement_line)
		else:
			latest_pos_line = _build_latest_position_line(friendly_latest)
			if latest_pos_line:
				lines.append(latest_pos_line)

		hp_ammo_line = _build_hp_ammo_line(_extract_friendly(first), friendly_latest)
		if hp_ammo_line:
			lines.append(hp_ammo_line)

		reason_line = _build_teammate_reason_line(friendly_latest, max_items=3)
		if reason_line:
			lines.append(reason_line)

		enemy_last_seen_line = _build_enemy_last_seen_line(entries, max_items=3)
		if enemy_last_seen_line:
			lines.append(enemy_last_seen_line)

		note_tail = [item.note for item in entries if item.note]
		if note_tail:
			lines.append("Recent notes: {}".format(" | ".join(note_tail[-3:])))

		if max_lines > 0:
			lines = lines[: int(max_lines)]
		return "\n".join(lines)


def _safe_state_copy(state: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
	if isinstance(state, Mapping):
		return copy.deepcopy(dict(state))
	return {}


def _extract_friendly(state: Mapping[str, Any]) -> Mapping[str, Any]:
	friendly = state.get("friendly", {})
	if isinstance(friendly, Mapping):
		return friendly
	return {}


def _extract_enemy(state: Mapping[str, Any]) -> Mapping[str, Any]:
	enemy = state.get("enemy", {})
	if isinstance(enemy, Mapping):
		return enemy
	return {}


def _extract_robot_state(entry: Any) -> Mapping[str, Any]:
	if not isinstance(entry, Mapping):
		return {}
	state = entry.get("state", {})
	if isinstance(state, Mapping):
		return state
	return {}


def _as_float(value: Any, default: float) -> float:
	try:
		return float(value)
	except (TypeError, ValueError):
		return float(default)


def _to_float_or_none(value: Any) -> Optional[float]:
	try:
		return float(value)
	except (TypeError, ValueError):
		return None


def _get_nested(value: Any, keys: Sequence[str]) -> Optional[Any]:
	cursor = value
	for key in keys:
		if not isinstance(cursor, Mapping):
			return None
		cursor = cursor.get(key)
	return cursor


def _find_position_component(state_map: Mapping[str, Any], key: str) -> Tuple[Optional[Any], bool]:
	candidates = (
		("pose", "position", key),
		("position", key),
		("pos", key),
		(key,),
	)
	for path in candidates:
		value = _get_nested(state_map, path)
		if value is not None:
			return value, True
	return None, False


def _extract_position(state_map: Mapping[str, Any]) -> Optional[Dict[str, float]]:
	x_raw, x_ok = _find_position_component(state_map, "x")
	y_raw, y_ok = _find_position_component(state_map, "y")
	if not (x_ok and y_ok):
		return None

	x = _to_float_or_none(x_raw)
	y = _to_float_or_none(y_raw)
	if x is None or y is None:
		return None

	yaw_raw, yaw_ok = _find_position_component(state_map, "yaw")
	yaw = _to_float_or_none(yaw_raw) if yaw_ok else None
	return {
		"x": x,
		"y": y,
		"yaw": yaw if yaw is not None else 0.0,
	}


def _count_visible_enemies(enemy_block: Mapping[str, Any]) -> int:
	if not isinstance(enemy_block, Mapping):
		return 0

	state = enemy_block.get("state", {})
	if not isinstance(state, Mapping):
		return 0

	visible = state.get("visible_enemies")
	if isinstance(visible, list):
		return len([item for item in visible if isinstance(item, Mapping)])

	enemies = state.get("enemies")
	if isinstance(enemies, list):
		return len([item for item in enemies if isinstance(item, Mapping) and item.get("visible", True)])

	if "x" in state and "y" in state and state.get("visible", True):
		return 1
	return 0


def _build_friendly_status_line(friendly_latest: Mapping[str, Any], visible_enemy_count: int) -> str:
	if not friendly_latest:
		return "Latest friendly: count=0 visible_enemies={}.".format(visible_enemy_count)

	alive_count = 0
	low_hp_count = 0
	in_combat_count = 0
	for entry in friendly_latest.values():
		state = _extract_robot_state(entry)
		alive = bool(state.get("alive", True))
		hp = _as_float(state.get("hp", 100.0), 100.0)
		in_combat = bool(state.get("in_combat", False))
		if alive and hp > 0:
			alive_count += 1
		if alive and hp <= 25.0:
			low_hp_count += 1
		if in_combat:
			in_combat_count += 1

	line = "Latest friendly: count={} alive={} low_hp={} visible_enemies={}.".format(
		len(friendly_latest),
		alive_count,
		low_hp_count,
		visible_enemy_count,
	)
	if in_combat_count:
		line = line[:-1] + " in_combat={}.".format(in_combat_count)
	return line


def _build_movement_line(
	friendly_first: Mapping[str, Any],
	friendly_last: Mapping[str, Any],
	max_items: int = 4,
) -> str:
	robot_ids = sorted(set(list(friendly_first.keys()) + list(friendly_last.keys())))
	if not robot_ids:
		return ""

	chunks: List[str] = []
	for robot_id in robot_ids:
		first_state = _extract_robot_state(friendly_first.get(robot_id, {}))
		last_state = _extract_robot_state(friendly_last.get(robot_id, {}))
		first_pos = _extract_position(first_state)
		last_pos = _extract_position(last_state)
		if not first_pos or not last_pos:
			continue

		dx = last_pos["x"] - first_pos["x"]
		dy = last_pos["y"] - first_pos["y"]
		chunks.append(
			"{} dx={:+.2f} dy={:+.2f} pos=({:.2f},{:.2f})".format(
				robot_id,
				dx,
				dy,
				last_pos["x"],
				last_pos["y"],
			)
		)

	if not chunks:
		return ""

	if max_items > 0 and len(chunks) > max_items:
		extra = len(chunks) - max_items
		chunks = chunks[:max_items]
		chunks.append("+{} more".format(extra))

	return "Movement (first->latest): {}.".format("; ".join(chunks))


def _build_latest_position_line(friendly_last: Mapping[str, Any], max_items: int = 4) -> str:
	if not friendly_last:
		return ""

	chunks: List[str] = []
	for robot_id in sorted(friendly_last.keys()):
		state = _extract_robot_state(friendly_last.get(robot_id, {}))
		pos = _extract_position(state)
		if not pos:
			continue
		chunks.append("{}({:.2f},{:.2f})".format(robot_id, pos["x"], pos["y"]))

	if not chunks:
		return ""
	if max_items > 0 and len(chunks) > max_items:
		extra = len(chunks) - max_items
		chunks = chunks[:max_items]
		chunks.append("+{} more".format(extra))

	return "Latest positions: {}.".format("; ".join(chunks))


def _build_hp_ammo_line(
	friendly_first: Mapping[str, Any],
	friendly_last: Mapping[str, Any],
	max_items: int = 4,
) -> str:
	robot_ids = sorted(set(list(friendly_first.keys()) + list(friendly_last.keys())))
	if not robot_ids:
		return ""

	chunks: List[str] = []
	for robot_id in robot_ids:
		first_state = _extract_robot_state(friendly_first.get(robot_id, {}))
		last_state = _extract_robot_state(friendly_last.get(robot_id, {}))
		hp_present = ("hp" in first_state) or ("hp" in last_state)
		ammo_present = "ammo" in last_state
		if not (hp_present or ammo_present):
			continue

		hp_first = _as_float(first_state.get("hp", 100.0), 100.0)
		hp_last = _as_float(last_state.get("hp", hp_first), hp_first)
		hp_delta = hp_last - hp_first
		ammo_last = _as_float(last_state.get("ammo", 0.0), 0.0)

		chunks.append("{} hp={:.0f}({:+.0f}) ammo={:.0f}".format(robot_id, hp_last, hp_delta, ammo_last))

	if not chunks:
		return ""
	if max_items > 0 and len(chunks) > max_items:
		extra = len(chunks) - max_items
		chunks = chunks[:max_items]
		chunks.append("+{} more".format(extra))

	return "HP/ammo (latest, delta): {}.".format("; ".join(chunks))


def _build_teammate_reason_line(
	friendly_latest: Mapping[str, Any],
	max_items: int = 3,
	max_reason_chars: int = 60,
) -> str:
	if not friendly_latest:
		return ""

	chunks: List[str] = []
	for robot_id in sorted(friendly_latest.keys()):
		state = _extract_robot_state(friendly_latest.get(robot_id, {}))
		reason = str(state.get("reason", "")).strip()
		if not reason:
			continue
		reason = _truncate_text(reason, max_reason_chars)
		chunks.append("{}:{}".format(robot_id, reason))

	if not chunks:
		return ""
	if max_items > 0 and len(chunks) > max_items:
		extra = len(chunks) - max_items
		chunks = chunks[:max_items]
		chunks.append("+{} more".format(extra))

	return "Teammate intent: {}.".format("; ".join(chunks))


def _extract_enemies_from_state(enemy_state: Mapping[str, Any]) -> List[Dict[str, Any]]:
	state = enemy_state.get("state", {})
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

	result: List[Dict[str, Any]] = []
	for idx, item in enumerate(items):
		if not isinstance(item, Mapping):
			continue
		if item.get("visible") is False:
			continue

		enemy_id = str(item.get("id", item.get("robot_ns", item.get("name", ""))))
		if not enemy_id:
			enemy_id = "enemy_{}".format(idx + 1)

		x_raw, x_ok = _find_position_component(item, "x")
		y_raw, y_ok = _find_position_component(item, "y")
		if not (x_ok and y_ok):
			continue
		x = _to_float_or_none(x_raw)
		y = _to_float_or_none(y_raw)
		if x is None or y is None:
			continue

		result.append({"id": enemy_id, "x": x, "y": y})

	return result


def _build_enemy_last_seen_line(entries: Sequence[STMEntry], max_items: int = 3) -> str:
	if not entries:
		return ""

	last_seen: Dict[str, Dict[str, Any]] = {}
	for entry in entries:
		enemy_state = _extract_enemy(entry.state)
		for enemy in _extract_enemies_from_state(enemy_state):
			enemy_id = str(enemy.get("id", "")).strip()
			if not enemy_id:
				continue
			last_seen[enemy_id] = {
				"id": enemy_id,
				"x": enemy.get("x"),
				"y": enemy.get("y"),
				"timestamp_s": entry.timestamp_s,
			}

	if not last_seen:
		return ""

	now_s = time.time()
	sorted_items = sorted(
		last_seen.values(),
		key=lambda item: float(item.get("timestamp_s", 0.0)),
		reverse=True,
	)

	chunks: List[str] = []
	for item in sorted_items[: max(0, int(max_items))]:
		x = _to_float_or_none(item.get("x"))
		y = _to_float_or_none(item.get("y"))
		if x is None or y is None:
			continue
		age = max(0.0, now_s - float(item.get("timestamp_s", now_s)))
		chunks.append(
			"{}({:.2f},{:.2f}) age={:.1f}s".format(
				item.get("id", "enemy"),
				x,
				y,
				age,
			)
		)

	if not chunks:
		return ""
	return "Enemy last seen: {}.".format("; ".join(chunks))


def _truncate_text(text: str, max_chars: int) -> str:
	value = str(text or "")
	if len(value) <= max_chars:
		return value
	if max_chars <= 3:
		return value[:max_chars]
	return value[: max_chars - 3] + "..."


__all__ = [
	"STMEntry",
	"ShortTermMemory",
]
