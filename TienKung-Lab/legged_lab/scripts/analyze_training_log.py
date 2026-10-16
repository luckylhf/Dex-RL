#!/usr/bin/env python3
"""Training log analyzer for dex_walk (and compatible tasks).

Usage:
    python analyze_training_log.py                    # latest run under logs/dex_walk
    python analyze_training_log.py logs/dex_walk/RUN  # specific run dir or event file
    python analyze_training_log.py --task dex_walk    # different task
    python analyze_training_log.py --watch            # live refresh every N seconds
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
EVENT_GLOB = "events.out.tfevents.*"


# ---------------------------------------------------------------------------
# Tag groups — ordered for display
# ---------------------------------------------------------------------------

GROUPS: list[tuple[str, list[str]]] = [
    ("training", [
        "Train/mean_reward",
        "Train/mean_episode_length",
        "Policy/mean_noise_std",
        "Curriculum/terrain_levels",
    ]),
    ("velocity_tracking", [
        "Episode_Reward/track_lin_vel_xy_exp",
        "Episode_Reward/track_ang_vel_z_exp",
        "Episode_Reward/lin_vel_z_l2",
        "Episode_Reward/ang_vel_xy_l2",
    ]),
    ("gait", [
        "Episode_Reward/gait_feet_frc_perio",
        "Episode_Reward/gait_feet_spd_perio",
        "Episode_Reward/gait_feet_frc_support_perio",
        "Episode_Reward/fly",
        "Episode_Reward/alive_reward",
    ]),
    ("feet", [
        "Episode_Reward/feet_slide",
        "Episode_Reward/feet_force",
        "Episode_Reward/feet_too_near",
        "Episode_Reward/feet_stumble",
        "Episode_Reward/feet_y_distance",
    ]),
    ("orientation", [
        "Episode_Reward/body_orientation_l2",
        "Episode_Reward/waist_orientation_l2",
        "Episode_Reward/torso_ang_vel_xy_l2",
        "Episode_Reward/torso_ang_acc_xy_l2",
        "Episode_Reward/flat_orientation_l2",
    ]),
    ("joints", [
        "Episode_Reward/joint_deviation_hip",
        "Episode_Reward/joint_deviation_arms",
        "Episode_Reward/joint_deviation_legs",
        "Episode_Reward/waist_joint_deviation_l1",
        "Episode_Reward/waist_joint_velocity",
        "Episode_Reward/hip_roll_action",
        "Episode_Reward/hip_yaw_action",
        "Episode_Reward/ankle_torque",
        "Episode_Reward/ankle_action",
        "Episode_Reward/dof_pos_limits",
    ]),
    ("energy", [
        "Episode_Reward/energy",
        "Episode_Reward/dof_acc_l2",
        "Episode_Reward/action_rate_l2",
    ]),
    ("stand_still", [
        "Episode_Reward/stand_still_exp",
        "Episode_Reward/stand_still_vel",
        "Episode_Reward/stand_still_feet_motion_penalty",
        "Episode_Reward/stand_still_double_support",
    ]),
    ("contacts", [
        "Episode_Reward/undesired_contacts",
        "Episode_Reward/termination_penalty",
    ]),
    ("losses", [
        "Loss/value_function",
        "Loss/surrogate",
        "Loss/entropy",
        "Loss/amp",
        "Loss/amp_policy_pred",
        "Loss/amp_expert_pred",
        "Loss/amp_grad_pen",
        "Loss/mirror",
        "Loss/symmetry",
        "Loss/learning_rate",
    ]),
    ("perf", [
        "Perf/total_fps",
        "Perf/collection time",
        "Perf/learning_time",
    ]),
]

# Short display names (strip group prefix for readability)
SHORT: dict[str, str] = {
    "Train/mean_reward":                        "reward",
    "Train/mean_episode_length":                "ep_len",
    "Policy/mean_noise_std":                    "noise_std",
    "Curriculum/terrain_levels":                "terrain_lvl",
    "Episode_Reward/track_lin_vel_xy_exp":      "track_lin",
    "Episode_Reward/track_ang_vel_z_exp":       "track_ang",
    "Episode_Reward/lin_vel_z_l2":              "lin_vel_z",
    "Episode_Reward/ang_vel_xy_l2":             "ang_vel_xy",
    "Episode_Reward/gait_feet_frc_perio":       "frc_perio",
    "Episode_Reward/gait_feet_spd_perio":       "spd_perio",
    "Episode_Reward/gait_feet_frc_support_perio": "frc_supp",
    "Episode_Reward/fly":                       "fly",
    "Episode_Reward/alive_reward":              "alive",
    "Episode_Reward/feet_slide":                "slide",
    "Episode_Reward/feet_force":                "force",
    "Episode_Reward/feet_too_near":             "too_near",
    "Episode_Reward/feet_stumble":              "stumble",
    "Episode_Reward/feet_y_distance":           "y_dist",
    "Episode_Reward/body_orientation_l2":       "body_ori",
    "Episode_Reward/waist_orientation_l2":      "waist_ori",
    "Episode_Reward/torso_ang_vel_xy_l2":       "torso_av",
    "Episode_Reward/torso_ang_acc_xy_l2":       "torso_aa",
    "Episode_Reward/flat_orientation_l2":       "flat_ori",
    "Episode_Reward/joint_deviation_hip":       "dev_hip",
    "Episode_Reward/joint_deviation_arms":      "dev_arms",
    "Episode_Reward/joint_deviation_legs":      "dev_legs",
    "Episode_Reward/waist_joint_deviation_l1":  "waist_dev",
    "Episode_Reward/waist_joint_velocity":      "waist_vel",
    "Episode_Reward/hip_roll_action":           "hip_roll",
    "Episode_Reward/hip_yaw_action":            "hip_yaw",
    "Episode_Reward/ankle_torque":              "ank_torq",
    "Episode_Reward/ankle_action":              "ank_act",
    "Episode_Reward/dof_pos_limits":            "jnt_lim",
    "Episode_Reward/energy":                    "energy",
    "Episode_Reward/dof_acc_l2":                "dof_acc",
    "Episode_Reward/action_rate_l2":            "act_rate",
    "Episode_Reward/stand_still_exp":           "ss_exp",
    "Episode_Reward/stand_still_vel":           "ss_vel",
    "Episode_Reward/stand_still_feet_motion_penalty": "ss_feet",
    "Episode_Reward/stand_still_double_support": "ss_dbl",
    "Episode_Reward/undesired_contacts":        "bad_cont",
    "Episode_Reward/termination_penalty":       "term_pen",
    "Loss/value_function":                      "val_loss",
    "Loss/surrogate":                           "surr_loss",
    "Loss/entropy":                             "entropy",
    "Loss/amp":                                 "amp",
    "Loss/amp_policy_pred":                     "amp_pol",
    "Loss/amp_expert_pred":                     "amp_exp",
    "Loss/amp_grad_pen":                        "amp_gpen",
    "Loss/mirror":                              "mirror",
    "Loss/symmetry":                            "symmetry",
    "Loss/learning_rate":                       "lr",
    "Perf/total_fps":                           "fps",
    "Perf/collection time":                     "coll_t",
    "Perf/learning_time":                       "learn_t",
}

# Chinese annotations shown in single-column mode (max 6 chars)
COMMENT: dict[str, str] = {
    "Train/mean_reward":                          "总奖励",
    "Train/mean_episode_length":                  "回合长度",
    "Policy/mean_noise_std":                      "动作噪声",
    "Curriculum/terrain_levels":                  "地形等级",
    "Episode_Reward/track_lin_vel_xy_exp":        "线速跟踪",
    "Episode_Reward/track_ang_vel_z_exp":         "角速跟踪",
    "Episode_Reward/lin_vel_z_l2":                "垂直速度",
    "Episode_Reward/ang_vel_xy_l2":               "横滚俯仰速",
    "Episode_Reward/gait_feet_frc_perio":         "步态力周期",
    "Episode_Reward/gait_feet_spd_perio":         "步态速周期",
    "Episode_Reward/gait_feet_frc_support_perio": "支撑力周期",
    "Episode_Reward/fly":                         "腾空惩罚",
    "Episode_Reward/alive_reward":                "存活奖励",
    "Episode_Reward/feet_slide":                  "脚底滑动",
    "Episode_Reward/feet_force":                  "脚底力",
    "Episode_Reward/feet_too_near":               "双脚过近",
    "Episode_Reward/feet_stumble":                "绊脚惩罚",
    "Episode_Reward/feet_y_distance":             "脚横向距离",
    "Episode_Reward/body_orientation_l2":         "躯干姿态",
    "Episode_Reward/waist_orientation_l2":        "腰部姿态",
    "Episode_Reward/torso_ang_vel_xy_l2":         "躯干角速度",
    "Episode_Reward/torso_ang_acc_xy_l2":         "躯干角加速",
    "Episode_Reward/flat_orientation_l2":         "水平姿态",
    "Episode_Reward/joint_deviation_hip":         "髋关节偏差",
    "Episode_Reward/joint_deviation_arms":        "臂关节偏差",
    "Episode_Reward/joint_deviation_legs":        "腿关节偏差",
    "Episode_Reward/waist_joint_deviation_l1":    "腰关节偏差",
    "Episode_Reward/waist_joint_velocity":        "腰关节速度",
    "Episode_Reward/hip_roll_action":             "髋滚动作",
    "Episode_Reward/hip_yaw_action":              "髋偏航动作",
    "Episode_Reward/ankle_torque":                "踝关节力矩",
    "Episode_Reward/ankle_action":                "踝关节动作",
    "Episode_Reward/dof_pos_limits":              "关节限位",
    "Episode_Reward/energy":                      "能量消耗",
    "Episode_Reward/dof_acc_l2":                  "关节加速度",
    "Episode_Reward/action_rate_l2":              "动作变化率",
    "Episode_Reward/stand_still_exp":             "静止奖励",
    "Episode_Reward/stand_still_vel":             "静止速度",
    "Episode_Reward/stand_still_feet_motion_penalty": "静止脚动",
    "Episode_Reward/stand_still_double_support":  "双脚支撑",
    "Episode_Reward/undesired_contacts":          "非期望接触",
    "Episode_Reward/termination_penalty":         "终止惩罚",
    "Loss/value_function":                        "价值损失",
    "Loss/surrogate":                             "策略损失",
    "Loss/entropy":                               "熵损失",
    "Loss/amp":                                   "判别器损失",
    "Loss/amp_policy_pred":                       "策略判别值",
    "Loss/amp_expert_pred":                       "专家判别值",
    "Loss/amp_grad_pen":                          "梯度惩罚",
    "Loss/mirror":                                "镜像损失",
    "Loss/symmetry":                              "对称损失",
    "Loss/learning_rate":                         "学习率",
    "Perf/total_fps":                             "总帧率",
    "Perf/collection time":                       "采集时间",
    "Perf/learning_time":                         "学习时间",
}

# Severity thresholds for penalty terms: (warn, critical)
# Values below warn → "!", below critical → "!!", otherwise empty
SEVERITY: dict[str, tuple[float, float]] = {
    "Episode_Reward/lin_vel_z_l2":              (-0.05,  -0.15),
    "Episode_Reward/ang_vel_xy_l2":             (-0.05,  -0.15),
    "Episode_Reward/body_orientation_l2":       (-0.2,   -0.5),
    "Episode_Reward/waist_orientation_l2":      (-0.2,   -0.5),
    "Episode_Reward/torso_ang_vel_xy_l2":       (-0.3,   -0.8),
    "Episode_Reward/torso_ang_acc_xy_l2":       (-0.05,  -0.15),
    "Episode_Reward/flat_orientation_l2":       (-0.02,  -0.05),
    "Episode_Reward/feet_slide":                (-0.02,  -0.08),
    "Episode_Reward/feet_force":                (-0.01,  -0.05),
    "Episode_Reward/feet_too_near":             (-0.02,  -0.1),
    "Episode_Reward/feet_stumble":              (-0.02,  -0.1),
    "Episode_Reward/feet_y_distance":           (-0.05,  -0.2),
    "Episode_Reward/joint_deviation_hip":       (-0.05,  -0.2),
    "Episode_Reward/joint_deviation_arms":      (-0.02,  -0.1),
    "Episode_Reward/joint_deviation_legs":      (-0.01,  -0.05),
    "Episode_Reward/waist_joint_deviation_l1":  (-0.05,  -0.2),
    "Episode_Reward/waist_joint_velocity":      (-0.03,  -0.1),
    "Episode_Reward/hip_roll_action":           (-0.15,  -0.4),
    "Episode_Reward/hip_yaw_action":            (-0.2,   -0.5),
    "Episode_Reward/ankle_torque":              (-0.05,  -0.15),
    "Episode_Reward/ankle_action":              (-0.005, -0.02),
    "Episode_Reward/dof_pos_limits":            (-0.02,  -0.1),
    "Episode_Reward/energy":                    (-0.02,  -0.05),
    "Episode_Reward/dof_acc_l2":                (-0.003, -0.01),
    "Episode_Reward/action_rate_l2":            (-0.01,  -0.03),
    "Episode_Reward/fly":                       (-0.02,  -0.1),
    "Episode_Reward/undesired_contacts":        (-0.02,  -0.1),
    "Episode_Reward/termination_penalty":       (-0.5,   -2.0),
    "Episode_Reward/stand_still_vel":           (-0.05,  -0.2),
    "Episode_Reward/stand_still_feet_motion_penalty": (-0.02, -0.1),
}

# Healthy direction: True = higher is better, False = lower is better
HIGHER_BETTER: dict[str, bool] = {
    "Train/mean_reward": True,
    "Train/mean_episode_length": True,
    "Episode_Reward/track_lin_vel_xy_exp": True,
    "Episode_Reward/track_ang_vel_z_exp": True,
    "Episode_Reward/gait_feet_frc_perio": True,
    "Episode_Reward/gait_feet_spd_perio": True,
    "Episode_Reward/gait_feet_frc_support_perio": True,
    "Episode_Reward/alive_reward": True,
    "Episode_Reward/stand_still_exp": True,
    "Episode_Reward/stand_still_double_support": True,
}


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Series:
    tag: str
    steps: list[int]
    values: list[float]

    @property
    def last_step(self) -> int:
        return self.steps[-1]

    @property
    def first(self) -> float:
        return self.values[0]

    @property
    def last(self) -> float:
        return self.values[-1]

    @property
    def best(self) -> float:
        return max(self.values) if HIGHER_BETTER.get(self.tag, False) else min(self.values)

    def window_mean(self, n: int) -> float:
        return _mean(self.values[-n:])

    def prev_window_mean(self, n: int) -> float:
        end = max(0, len(self.values) - n)
        start = max(0, end - n)
        return _mean(self.values[start:end]) if end > start else self.window_mean(n)

    def trend(self, n: int) -> float:
        return self.window_mean(n) - self.prev_window_mean(n)


def _mean(values: Iterable[float]) -> float:
    lst = list(values)
    return sum(lst) / len(lst) if lst else 0.0


def _dw(s: str) -> int:
    """Display width: CJK chars count as 2, others as 1."""
    import unicodedata
    w = 0
    for c in s:
        ea = unicodedata.east_asian_width(c)
        w += 2 if ea in ("W", "F") else 1
    return w


def _ljust_dw(s: str, width: int) -> str:
    return s + " " * max(0, width - _dw(s))


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def latest_event_file(search_root: Path) -> Path:
    candidates = [p for p in search_root.rglob(EVENT_GLOB) if p.is_file()]
    if not candidates:
        raise FileNotFoundError(f"No TensorBoard event files found under: {search_root}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def resolve_event_file(target: Path | None, task: str) -> Path:
    if target is None:
        return latest_event_file(WORKSPACE_ROOT / "logs" / task)
    resolved = target.expanduser().resolve()
    if resolved.is_file():
        return resolved
    if resolved.is_dir():
        return latest_event_file(resolved)
    raise FileNotFoundError(f"Path does not exist: {target}")


def load_series(event_file: Path) -> dict[str, Series]:
    from tensorboard.backend.event_processing import event_accumulator as ea
    acc = ea.EventAccumulator(str(event_file), size_guidance={"scalars": 0})
    acc.Reload()
    result: dict[str, Series] = {}
    for tag in acc.Tags().get("scalars", []):
        events = acc.Scalars(tag)
        if events:
            result[tag] = Series(
                tag=tag,
                steps=[int(e.step) for e in events],
                values=[float(e.value) for e in events],
            )
    return result


# Tags where value can be displayed as percentage.
# Value: (max_value, description)
# max_value = weight * raw_max
PCT_TAGS: dict[str, float] = {
    # Positive rewards with known max
    "Episode_Reward/track_lin_vel_xy_exp":        5.0 * 0.6,   # weight=5.0, gated non-zero cmd, 60% walking
    "Episode_Reward/track_ang_vel_z_exp":         2.5 * 0.6,   # weight=2.5, gated non-zero cmd, 60% walking
    "Episode_Reward/alive_reward":                1.0,   # weight=1.0, raw in [0,1]
    "Episode_Reward/stand_still_exp":             7.0 * 0.4,   # weight=7.0, raw in [0,1]
    "Episode_Reward/stand_still_double_support":  3.0 * 0.4,  # weight=3.0, raw in [0,1], 40% standing
    "Episode_Reward/gait_feet_frc_perio":         2.0,   # weight=2.0, raw in [0,2] (2 feet)
    "Episode_Reward/gait_feet_frc_support_perio": 1.5,   # weight=1.5, raw in [0,2]
    "Episode_Reward/gait_feet_spd_perio":         2.0,   # weight=2.0, raw in [0,2]
}

# ep_len max for percentage display
EP_LEN_MAX = 1000  # max_episode_length_s=20.0 / (dt=0.005 * decimation=4) = 1000 steps


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

def _fmt(v: float, tag: str = "") -> str:
    if abs(v) >= 1000:
        return f"{v:.0f}"
    if abs(v) >= 100:
        return f"{v:.1f}"
    if abs(v) >= 10:
        return f"{v:.2f}"
    return f"{v:.3f}"


def _severity(tag: str, value: float) -> str:
    thresholds = SEVERITY.get(tag)
    if thresholds is None:
        return ""
    warn, critical = thresholds
    if value <= critical:
        return "!!"
    if value <= warn:
        return "!"
    return ""


def _trend_arrow(delta: float, tag: str) -> str:
    if abs(delta) < 1e-6:
        return "~"
    up = delta > 0
    good = HIGHER_BETTER.get(tag, False)
    if up:
        return "↑" if good else "↑!"
    return "↓!" if good else "↓"


def _bar(v: float, lo: float, hi: float, width: int = 8) -> str:
    if hi <= lo:
        return " " * width
    frac = max(0.0, min(1.0, (v - lo) / (hi - lo)))
    filled = round(frac * width)
    return "█" * filled + "░" * (width - filled)


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------

def print_header(event_file: Path, series_map: dict[str, Series], max_iterations: int = 50000):
    any_series = next(iter(series_map.values()), None)
    step = any_series.last_step if any_series else 0
    run_dir = event_file.parent.name
    pct = step / max_iterations * 100
    bar = _bar(step, 0, max_iterations, width=30)
    print(f"  lrs_walk  |  run: {run_dir}   \n  step: {step} / {max_iterations}  [{bar}]  {pct:.1f}%")
    print(f"{'='*60}")


def _percentile_value(s: "Series", pct: float) -> float:
    """Return the value at the given percentile (0.0, 0.5, 1.0) of the series."""
    idx = min(len(s.values) - 1, max(0, round((len(s.values) - 1) * pct)))
    return s.values[idx]


def _percentile_step(s: "Series", pct: float) -> int:
    idx = min(len(s.steps) - 1, max(0, round((len(s.steps) - 1) * pct)))
    return s.steps[idx]


def _slot_width(widths: list[int]) -> int:
    """Total character width of one table slot (columns + separators)."""
    return sum(widths) + 3 * (len(widths) - 1)  # 3 chars per " | " separator


def _auto_cols(tags: list[str], series_map: dict[str, Series]) -> int:
    """Pick 2/3 columns based on terminal width."""
    import shutil
    term_w = shutil.get_terminal_size((200, 40)).columns

    # Compute the actual max width per column across all tags (same logic as print_flat_table)
    hdr = ["metric", "注释", "000", "000", "000", "00.0%", "↑"]
    sample_widths = [len(h) for h in hdr]
    for tag in tags:
        s = series_map.get(tag)
        if s is None:
            continue
        short = SHORT.get(tag, tag.split("/")[-1])
        sample_widths[0] = max(sample_widths[0], len(short))
        sample_widths[1] = max(sample_widths[1], _dw(COMMENT.get(tag, "")))
        for col_idx, pct in enumerate([0.0, 0.5, 1.0]):
            idx = min(int(pct * (len(s.values) - 1)), len(s.values) - 1)
            v = _fmt(s.values[idx], tag)
            sample_widths[col_idx + 2] = max(sample_widths[col_idx + 2], len(v))
        # % column
        cur = s.values[-1]
        if tag in PCT_TAGS:
            pct_str = f"{cur / PCT_TAGS[tag] * 100:.1f}%"
        elif tag == "Train/mean_episode_length":
            pct_str = f"{cur / EP_LEN_MAX * 100:.1f}%"
        else:
            pct_str = ""
        sample_widths[5] = max(sample_widths[5], len(pct_str))
        # trend arrow is always 1-2 chars, hdr already covers it

    # slot width = sum of col widths + 3 chars per " | " between cols
    slot_w = sum(sample_widths) + 3 * (len(sample_widths) - 1)
    for cols in [3, 2]:
        total = cols * slot_w + (cols - 1) * 3  # cols slots + (cols-1) SEPs between slots
        if total <= term_w:
            return cols
    return 1


def print_flat_table(tags: list[str], series_map: dict[str, Series], window: int, cols: int = 0):
    """Single flat table, auto-selects 2/3/4 columns based on terminal width."""
    if cols == 0:
        cols = _auto_cols(tags, series_map)

    ref_tag = max((t for t in tags if t in series_map), key=lambda t: len(series_map[t].steps), default=None)
    if ref_tag is None:
        return
    dataIdx = [0.8,0.9,1.0]
    ref = series_map[ref_tag]
    s0   = _percentile_step(ref, dataIdx[0])
    s50  = _percentile_step(ref, dataIdx[1])
    s100 = _percentile_step(ref, dataIdx[2])

    hdr = ["metric", "注释", f"{s0}", f"{s50}", f"{s100}", "%/!", "↑"]

    cells: list[list[str]] = []
    for tag in tags:
        s = series_map.get(tag)
        if s is None:
            continue
        short = SHORT.get(tag, tag.split("/")[-1])
        v0   = _fmt(_percentile_value(s, dataIdx[0]), tag)
        v50  = _fmt(_percentile_value(s, dataIdx[1]), tag)
        v100 = _fmt(_percentile_value(s, dataIdx[2]), tag)
        cur = _percentile_value(s, 1.0)
        if tag in PCT_TAGS:
            pct_sev = f"{cur / PCT_TAGS[tag] * 100:.1f}%"
        elif tag == "Train/mean_episode_length":
            pct_sev = f"{cur / EP_LEN_MAX * 100:.1f}%"
        else:
            pct_sev = _severity(tag, cur)
        arrow = _trend_arrow(s.trend(window), tag)
        cells.append([short, COMMENT.get(tag, ""), v0, v50, v100, pct_sev, arrow])

    if not cells:
        return

    n_cols = len(hdr)
    while len(cells) % cols:
        cells.append([""] * n_cols)

    # per-slot column widths (use display width for CJK-safe alignment)
    slot_w: list[list[int]] = [[_dw(hdr[j]) for j in range(n_cols)] for _ in range(cols)]
    for i, cell in enumerate(cells):
        slot = i % cols
        for j, val in enumerate(cell):
            slot_w[slot][j] = max(slot_w[slot][j], _dw(val))

    SEP = "   "
    COL_JOIN = " | "
    SEP_JOIN = "-+-"
    # columns that are left-justified (text); col 1 is comment only in single-col mode
    left_cols = {0, 1}

    def fmt_row(row: list[str], widths: list[int]) -> str:
        parts = []
        for j, val in enumerate(row):
            if j in left_cols:
                parts.append(_ljust_dw(val, widths[j]))
            else:
                parts.append(val.rjust(widths[j]))
        return COL_JOIN.join(parts)

    def sep_row(widths: list[int]) -> str:
        return SEP_JOIN.join("-" * widths[j] for j in range(n_cols))

    hdr_line = SEP.join(fmt_row(hdr, slot_w[c]) for c in range(cols))
    sep_line  = SEP.join(sep_row(slot_w[c]) for c in range(cols))
    print(hdr_line)
    print(sep_line)

    for row_start in range(0, len(cells), cols):
        row_cells = cells[row_start:row_start + cols]
        print(SEP.join(fmt_row(row_cells[c], slot_w[c]) for c in range(cols)))


def print_health(series_map: dict[str, Series], window: int):
    """One-line health summary with key indicators."""
    def get(tag):
        s = series_map.get(tag)
        return s.last if s else None

    ep_len = get("Train/mean_episode_length")
    reward = get("Train/mean_reward")
    noise = get("Policy/mean_noise_std")
    terrain = get("Curriculum/terrain_levels")
    track = get("Episode_Reward/track_lin_vel_xy_exp")
    waist_ori = get("Episode_Reward/waist_orientation_l2")
    gait = get("Episode_Reward/gait_feet_frc_perio")
    fly = get("Episode_Reward/fly")

    parts = []
    if ep_len is not None:
        bar = _bar(ep_len, 0, EP_LEN_MAX)
        parts.append(f"ep_len={ep_len:.0f} [{bar}]")
    if reward is not None:
        parts.append(f"reward={reward:.2f}")
    if track is not None:
        parts.append(f"track_lin={track:.3f}")
    if gait is not None:
        parts.append(f"gait={gait:.3f}")
    if waist_ori is not None:
        flag = " ⚠" if waist_ori < -0.2 else ""
        parts.append(f"waist_ori={waist_ori:.3f}{flag}")
    if fly is not None:
        flag = " ⚠" if fly < -0.05 else ""
        parts.append(f"fly={fly:.3f}{flag}")
    if noise is not None:
        parts.append(f"noise={noise:.3f}")
    if terrain is not None:
        parts.append(f"terrain={terrain:.2f}")

    print(f"\n  health: {' | '.join(parts)}")

    # AMP discriminator health
    pol_s = series_map.get("Loss/amp_policy_pred")
    exp_s = series_map.get("Loss/amp_expert_pred")
    amp_s = series_map.get("Loss/amp")
    if pol_s is not None and exp_s is not None:
        pol_v, exp_v = pol_s.last, exp_s.last
        disc_gap = exp_v - pol_v  # discriminator separation; healthy disc ≈ 2.0, collapsed ≈ 0
        # Bar tracks policy imitation progress: pol_v -1 (not mimicking) → +1 (mimicking expert)
        amp_bar = _bar(pol_v, -1.0, 1.0)
        if disc_gap < 0.3:
            status = "disc_collapsed ⚠"
        elif pol_v > 0.5:
            status = "policy_winning"
        elif pol_v > -0.3:
            status = "learning"
        else:
            status = "early"
        amp_reward_str = f"  disc_loss={amp_s.last:.3f}" if amp_s else ""
        print(f"  amp:    pol={pol_v:.3f} exp={exp_v:.3f} gap={disc_gap:.2f} [{amp_bar}]  → {status}{amp_reward_str}")

    # Trend assessment
    ep_s = series_map.get("Train/mean_episode_length")
    reward_s = series_map.get("Train/mean_reward")
    if ep_s and len(ep_s.values) >= window * 2:
        ep_trend = ep_s.trend(window)
        rew_trend = reward_s.trend(window) if reward_s else 0.0
        if ep_trend > 5:
            status = "improving"
        elif ep_trend < -5:
            status = "degrading ⚠"
        elif abs(ep_trend) < 1 and abs(rew_trend) < 0.05:
            status = "plateau"
        else:
            status = "stable"
        print(f"  trend:  ep_len_delta={ep_trend:+.1f}  reward_delta={rew_trend:+.3f}  → {status}")


def print_amp_health(series_map: dict[str, Series]):
    """AMP discriminator health check."""
    pol = series_map.get("Loss/amp_policy_pred")
    exp = series_map.get("Loss/amp_expert_pred")
    if pol is None or exp is None:
        return
    # Healthy: policy_pred → -1 (early) → +1 (policy mimicking expert); expert_pred → +1
    pol_v, exp_v = pol.last, exp.last
    disc_gap = exp_v - pol_v  # discriminator separation ≈ 2.0 early, decreases as policy improves
    if disc_gap < 0.3:
        status = "disc_collapsed ⚠"
    elif pol_v > 0.5:
        status = "policy_winning"
    elif pol_v > -0.3:
        status = "learning"
    else:
        status = "early"
    print(f"  amp:    policy_pred={pol_v:.3f}  expert_pred={exp_v:.3f}  gap={disc_gap:.3f}  → {status}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_once(event_file: Path, window: int, groups: list[str] | None):
    series_map = load_series(event_file)
    if not series_map:
        print("No scalar data found.")
        return

    print_header(event_file, series_map)
    print_health(series_map, window)
    print()

    # Collect all tags in display order (flat, no grouping)
    seen: set[str] = set()
    ordered_tags: list[str] = []
    active_groups = GROUPS if not groups else [(n, t) for n, t in GROUPS if n in groups]
    for _, tags in active_groups:
        for tag in tags:
            if tag not in seen:
                seen.add(tag)
                ordered_tags.append(tag)

    print_flat_table(ordered_tags, series_map, window, cols=0)


def main():
    parser = argparse.ArgumentParser(description="Analyze dex_walk training logs.")
    parser.add_argument(
        "event_file", nargs="?", type=Path,
        help="Event file or run directory. Defaults to latest under logs/<task>.",
    )
    parser.add_argument("--task", default="dex_walk", help="Task name for default log path (default: dex_walk).")
    parser.add_argument("--window", type=int, default=10, help="Trailing window size for trend (default: 10).")
    parser.add_argument(
        "--groups", nargs="*",
        choices=[n for n, _ in GROUPS],
        help="Show only these groups. Default: all.",
    )
    parser.add_argument(
        "--watch", type=float, default=0.0, metavar="SECONDS",
        help="Live refresh interval in seconds (e.g. --watch 30). 0 = run once.",
    )
    args = parser.parse_args()

    event_file = resolve_event_file(args.event_file, args.task)

    if args.watch > 0:
        try:
            while True:
                os.system("clear")
                run_once(event_file, args.window, args.groups)
                print(f"  [refreshing every {args.watch:.0f}s — Ctrl+C to stop]")
                time.sleep(args.watch)
        except KeyboardInterrupt:
            pass
    else:
        run_once(event_file, args.window, args.groups)


if __name__ == "__main__":
    main()
