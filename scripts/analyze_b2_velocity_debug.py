#!/usr/bin/env python3
"""Offline analysis for B2 deployment debug CSV.

This script:
1. Reads `b2_velocity_debug.csv`
2. Restores semantic names and physical meanings for each column
3. Generates an HTML report with SVG plots
4. Optionally creates a W&B offline run/artifact for later sync
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Iterable


LEG_ORDER = ["FL", "FR", "RL", "RR"]
JOINT_ORDER = ["hip", "thigh", "calf"]


@dataclass
class ColumnSpec:
    source_column: str
    semantic_name: str
    unit: str
    group: str
    meaning: str


def build_obs_specs() -> list[ColumnSpec]:
    specs: list[ColumnSpec] = []
    obs_idx = 0

    def add(name: str, unit: str, group: str, meaning: str) -> None:
        nonlocal obs_idx
        specs.append(
            ColumnSpec(
                source_column=f"obs_obs_{obs_idx}",
                semantic_name=name,
                unit=unit,
                group=group,
                meaning=meaning,
            )
        )
        obs_idx += 1

    add("base_vx", "m/s", "base_linear_velocity", "Robot base linear velocity along body-frame forward axis.")
    add("base_vy", "m/s", "base_linear_velocity", "Robot base linear velocity along body-frame left axis.")
    add("base_vz", "m/s", "base_linear_velocity", "Robot base linear velocity along body-frame up axis.")

    add("base_wx", "rad/s", "base_angular_velocity", "Robot base angular velocity around body-frame roll axis.")
    add("base_wy", "rad/s", "base_angular_velocity", "Robot base angular velocity around body-frame pitch axis.")
    add("base_wz", "rad/s", "base_angular_velocity", "Robot base angular velocity around body-frame yaw axis.")

    add("gravity_x_b", "unitless", "projected_gravity", "Gravity direction projected into the body frame, x component.")
    add("gravity_y_b", "unitless", "projected_gravity", "Gravity direction projected into the body frame, y component.")
    add("gravity_z_b", "unitless", "projected_gravity", "Gravity direction projected into the body frame, z component.")

    for leg in LEG_ORDER:
        for joint in JOINT_ORDER:
            add(
                f"{leg}_{joint}_pos_rel",
                "rad",
                "joint_positions",
                f"{leg} leg {joint} joint angle relative to the deploy default pose.",
            )

    for leg in LEG_ORDER:
        for joint in JOINT_ORDER:
            add(
                f"{leg}_{joint}_vel",
                "rad/s",
                "joint_velocities",
                f"{leg} leg {joint} joint angular velocity.",
            )

    for leg in LEG_ORDER:
        for joint in JOINT_ORDER:
            add(
                f"{leg}_{joint}_last_raw_action",
                "unitless",
                "actions",
                f"Previous policy raw output for {leg} {joint}; this is the network output before scale/offset/clip.",
            )

    add("cmd_vx", "m/s", "commands", "Commanded forward velocity from joystick or teleop input.")
    add("cmd_vy", "m/s", "commands", "Commanded lateral velocity from joystick or teleop input.")
    add("cmd_wz", "rad/s", "commands", "Commanded yaw angular velocity from joystick or teleop input.")

    return specs


def build_action_specs(prefix: str, processed: bool) -> list[ColumnSpec]:
    specs: list[ColumnSpec] = []
    for leg in LEG_ORDER:
        for joint in JOINT_ORDER:
            semantic_name = f"{leg}_{joint}_{prefix}"
            if processed:
                meaning = (
                    f"Processed target joint position command for {leg} {joint}. "
                    "This is the action after scale, offset and clip, so it is the actual position target sent to the low-level PD controller."
                )
                unit = "rad"
                group = "processed_action"
            else:
                meaning = (
                    f"Current policy raw output for {leg} {joint} before scale, offset and clip."
                )
                unit = "unitless"
                group = "raw_action"
            specs.append(
                ColumnSpec(
                    source_column=f"{group}_{len(specs)}",
                    semantic_name=semantic_name,
                    unit=unit,
                    group=group,
                    meaning=meaning,
                )
            )
    return specs


OBS_SPECS = build_obs_specs()
RAW_ACTION_SPECS = build_action_specs("raw_action", processed=False)
PROCESSED_ACTION_SPECS = build_action_specs("target_pos", processed=True)


def infer_schema(header: list[str]) -> list[ColumnSpec]:
    if len(header) != 73:
        raise ValueError(f"Unexpected column count {len(header)}. Expected 73 columns for B2 velocity debug CSV.")

    specs: list[ColumnSpec] = [
        ColumnSpec("step", "step", "index", "meta", "Control step index logged by the deploy loop.")
    ]

    specs.extend(OBS_SPECS)
    specs.extend(
        ColumnSpec(f"raw_action_{i}", spec.semantic_name, spec.unit, spec.group, spec.meaning)
        for i, spec in enumerate(RAW_ACTION_SPECS)
    )
    specs.extend(
        ColumnSpec(f"processed_action_{i}", spec.semantic_name, spec.unit, spec.group, spec.meaning)
        for i, spec in enumerate(PROCESSED_ACTION_SPECS)
    )

    expected = [spec.source_column for spec in specs]
    if header != expected:
        raise ValueError(
            "CSV header does not match the expected B2 schema.\n"
            f"Expected: {expected}\nGot: {header}"
        )
    return specs


def load_csv(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header.")
        rows: list[dict[str, float]] = []
        for row in reader:
            rows.append({k: float(v) for k, v in row.items() if k is not None})
    return reader.fieldnames, rows


def downsample(rows: list[dict[str, float]], max_points: int) -> list[dict[str, float]]:
    if len(rows) <= max_points:
        return rows
    stride = math.ceil(len(rows) / max_points)
    return rows[::stride]


def series(rows: list[dict[str, float]], key: str) -> list[float]:
    return [row[key] for row in rows]


def svg_line_plot(
    xs: list[float],
    named_series: list[tuple[str, list[float], str]],
    title: str,
    x_label: str,
    y_label: str,
    width: int = 1200,
    height: int = 360,
) -> str:
    margin_left = 72
    margin_right = 20
    margin_top = 30
    margin_bottom = 46

    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    all_values = [v for _, vals, _ in named_series for v in vals]
    y_min = min(all_values)
    y_max = max(all_values)
    if math.isclose(y_min, y_max):
        pad = 1.0 if abs(y_min) < 1e-6 else abs(y_min) * 0.1
        y_min -= pad
        y_max += pad

    x_min = min(xs)
    x_max = max(xs)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0

    def sx(x: float) -> float:
        return margin_left + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y: float) -> float:
        return margin_top + (y_max - y) / (y_max - y_min) * plot_h

    grid_lines = []
    for i in range(5):
        frac = i / 4
        y = margin_top + frac * plot_h
        y_val = y_max - frac * (y_max - y_min)
        grid_lines.append(
            f'<line x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + plot_w}" y2="{y:.2f}" stroke="#e5e7eb" stroke-width="1"/>'
            f'<text x="{margin_left - 8}" y="{y + 4:.2f}" text-anchor="end" font-size="11" fill="#374151">{y_val:.3f}</text>'
        )

    x_ticks = []
    for i in range(6):
        frac = i / 5
        x = margin_left + frac * plot_w
        x_val = x_min + frac * (x_max - x_min)
        x_ticks.append(
            f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" y2="{margin_top + plot_h}" stroke="#f3f4f6" stroke-width="1"/>'
            f'<text x="{x:.2f}" y="{margin_top + plot_h + 18}" text-anchor="middle" font-size="11" fill="#374151">{x_val:.2f}</text>'
        )

    lines = []
    legend = []
    for idx, (name, ys, color) in enumerate(named_series):
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(xs, ys))
        lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{points}"/>')
        legend_x = margin_left + idx * 180
        legend.append(
            f'<line x1="{legend_x}" y1="16" x2="{legend_x + 18}" y2="16" stroke="{color}" stroke-width="3"/>'
            f'<text x="{legend_x + 24}" y="20" font-size="12" fill="#111827">{html.escape(name)}</text>'
        )

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="{width}" height="{height}" fill="white"/>
  <text x="{margin_left}" y="20" font-size="16" font-weight="bold" fill="#111827">{html.escape(title)}</text>
  {''.join(legend)}
  <rect x="{margin_left}" y="{margin_top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#9ca3af"/>
  {''.join(grid_lines)}
  {''.join(x_ticks)}
  {''.join(lines)}
  <text x="{margin_left + plot_w / 2:.2f}" y="{height - 8}" text-anchor="middle" font-size="12" fill="#111827">{html.escape(x_label)}</text>
  <text x="18" y="{margin_top + plot_h / 2:.2f}" text-anchor="middle" font-size="12" fill="#111827" transform="rotate(-90 18 {margin_top + plot_h / 2:.2f})">{html.escape(y_label)}</text>
</svg>"""


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def summarize(rows: list[dict[str, float]], keys: Iterable[str]) -> dict[str, dict[str, float]]:
    summary: dict[str, dict[str, float]] = {}
    for key in keys:
        values = series(rows, key)
        summary[key] = {
            "min": min(values),
            "max": max(values),
            "mean": mean(values),
        }
    return summary


def build_report(
    rows: list[dict[str, float]],
    specs: list[ColumnSpec],
    output_dir: Path,
    step_dt: float,
    source_csv: Path,
) -> None:
    sampled_rows = downsample(rows, 2000)
    xs = [row["step"] * step_dt for row in sampled_rows]
    colors = ["#2563eb", "#dc2626", "#059669", "#7c3aed", "#ea580c", "#0891b2"]

    plots: list[tuple[str, str, str]] = []

    plot_defs = [
        (
            "base_linear_velocity.svg",
            "Base Linear Velocity",
            "Body-frame translational velocity of the trunk. `vx` is forward, `vy` is left, `vz` is upward.",
            [("base_vx", colors[0]), ("base_vy", colors[1]), ("base_vz", colors[2])],
            "Velocity (m/s)",
        ),
        (
            "base_angular_velocity.svg",
            "Base Angular Velocity",
            "Body-frame rotational speed of the trunk. `wx/wy/wz` correspond to roll, pitch, yaw rates.",
            [("base_wx", colors[0]), ("base_wy", colors[1]), ("base_wz", colors[2])],
            "Angular velocity (rad/s)",
        ),
        (
            "commands_vs_motion.svg",
            "Command vs Motion",
            "Joystick/teleop commands compared to realized body motion. This helps check command tracking and saturation.",
            [("cmd_vx", colors[3]), ("base_vx", colors[0]), ("cmd_wz", colors[4]), ("base_wz", colors[1])],
            "Mixed units (m/s, rad/s)",
        ),
        (
            "gravity_projection.svg",
            "Projected Gravity",
            "Gravity vector expressed in the body frame. Near `[0, 0, -1]` usually means the body is close to upright.",
            [("gravity_x_b", colors[0]), ("gravity_y_b", colors[1]), ("gravity_z_b", colors[2])],
            "Unitless",
        ),
        (
            "joint_pos_rel.svg",
            "Joint Position Error Relative to Default Pose",
            "Each curve is a joint angle minus the deploy default pose. Large offsets indicate crouching, extension, or posture asymmetry.",
            [(f"{leg}_{joint}_pos_rel", colors[(leg_i + joint_i) % len(colors)]) for leg_i, leg in enumerate(LEG_ORDER) for joint_i, joint in enumerate(JOINT_ORDER)],
            "Angle (rad)",
        ),
        (
            "joint_velocities.svg",
            "Joint Velocities",
            "Angular velocity of each leg joint. Peaks here often align with aggressive recovery or impact events.",
            [(f"{leg}_{joint}_vel", colors[(leg_i + joint_i) % len(colors)]) for leg_i, leg in enumerate(LEG_ORDER) for joint_i, joint in enumerate(JOINT_ORDER)],
            "Angular velocity (rad/s)",
        ),
        (
            "policy_actions.svg",
            "Policy Raw Actions",
            "Dimensionless network outputs before scale, offset, and clip. This is what the policy itself is trying to emit.",
            [(f"{leg}_{joint}_raw_action", colors[(leg_i + joint_i) % len(colors)]) for leg_i, leg in enumerate(LEG_ORDER) for joint_i, joint in enumerate(JOINT_ORDER)],
            "Unitless",
        ),
        (
            "target_positions.svg",
            "Processed Joint Target Positions",
            "Joint position targets actually sent after action scaling, offsetting, and clipping.",
            [(f"{leg}_{joint}_target_pos", colors[(leg_i + joint_i) % len(colors)]) for leg_i, leg in enumerate(LEG_ORDER) for joint_i, joint in enumerate(JOINT_ORDER)],
            "Angle (rad)",
        ),
    ]

    for file_name, title, description, key_defs, y_label in plot_defs:
        svg = svg_line_plot(
            xs,
            [(name, series(sampled_rows, name), color) for name, color in key_defs],
            title=title,
            x_label="Time (s)",
            y_label=y_label,
        )
        svg_path = output_dir / file_name
        write_text(svg_path, svg)
        plots.append((file_name, title, description))

    semantics_rows = []
    for spec in specs:
        semantics_rows.append(
            f"<tr><td>{html.escape(spec.source_column)}</td><td>{html.escape(spec.semantic_name)}</td><td>{html.escape(spec.group)}</td><td>{html.escape(spec.unit)}</td><td>{html.escape(spec.meaning)}</td></tr>"
        )

    report_sections = []
    for file_name, title, description in plots:
        report_sections.append(
            f"<section><h2>{html.escape(title)}</h2><p>{html.escape(description)}</p><img src=\"{html.escape(file_name)}\" style=\"width:100%; border:1px solid #d1d5db;\"/></section>"
        )

    report = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>B2 Velocity Debug Report</title>
  <style>
    body {{ font-family: sans-serif; margin: 24px auto; max-width: 1280px; color: #111827; line-height: 1.5; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border: 1px solid #d1d5db; padding: 6px 8px; text-align: left; vertical-align: top; }}
    th {{ background: #f3f4f6; }}
    code {{ background: #f3f4f6; padding: 1px 4px; border-radius: 4px; }}
    section {{ margin: 28px 0; }}
  </style>
</head>
<body>
  <h1>B2 Velocity Debug Offline Report</h1>
  <p>Source CSV: <code>{html.escape(str(source_csv))}</code></p>
  <p>Assumed control step: <code>{step_dt:.4f} s</code></p>
  <p>Joint order assumption for B2 policy space: <code>FL, FR, RL, RR</code> with per-leg joint order <code>hip, thigh, calf</code>. This ordering is inferred from the deployed <code>joint_ids_map</code> and the sign pattern of <code>default_joint_pos</code>.</p>
  <section>
    <h2>Column Meanings</h2>
    <table>
      <thead>
        <tr><th>CSV Column</th><th>Semantic Name</th><th>Group</th><th>Unit</th><th>Physical Meaning</th></tr>
      </thead>
      <tbody>
        {''.join(semantics_rows)}
      </tbody>
    </table>
  </section>
  {''.join(report_sections)}
</body>
</html>"""
    write_text(output_dir / "report.html", report)


def get_plot_defs() -> list[tuple[str, str, str, list[str], str]]:
    return [
        (
            "base_linear_velocity",
            "Base Linear Velocity",
            "Body-frame translational velocity of the trunk. vx is forward, vy is left, vz is upward.",
            ["base_vx", "base_vy", "base_vz"],
            "Velocity (m/s)",
        ),
        (
            "base_angular_velocity",
            "Base Angular Velocity",
            "Body-frame rotational speed of the trunk. wx/wy/wz correspond to roll, pitch, yaw rates.",
            ["base_wx", "base_wy", "base_wz"],
            "Angular velocity (rad/s)",
        ),
        (
            "commands_vs_motion",
            "Command vs Motion",
            "Joystick/teleop commands compared to realized body motion.",
            ["cmd_vx", "base_vx", "cmd_wz", "base_wz"],
            "Mixed units (m/s, rad/s)",
        ),
        (
            "gravity_projection",
            "Projected Gravity",
            "Gravity vector expressed in the body frame.",
            ["gravity_x_b", "gravity_y_b", "gravity_z_b"],
            "Unitless",
        ),
        (
            "joint_pos_rel",
            "Joint Position Error Relative to Default Pose",
            "Joint angle minus default standing pose for every B2 leg joint.",
            [f"{leg}_{joint}_pos_rel" for leg in LEG_ORDER for joint in JOINT_ORDER],
            "Angle (rad)",
        ),
        (
            "joint_velocities",
            "Joint Velocities",
            "Angular velocity of each leg joint.",
            [f"{leg}_{joint}_vel" for leg in LEG_ORDER for joint in JOINT_ORDER],
            "Angular velocity (rad/s)",
        ),
        (
            "policy_actions",
            "Policy Raw Actions",
            "Dimensionless network outputs before scale, offset, and clip.",
            [f"{leg}_{joint}_raw_action" for leg in LEG_ORDER for joint in JOINT_ORDER],
            "Unitless",
        ),
        (
            "target_positions",
            "Processed Joint Target Positions",
            "Joint position targets after scale, offset, and clip.",
            [f"{leg}_{joint}_target_pos" for leg in LEG_ORDER for joint in JOINT_ORDER],
            "Angle (rad)",
        ),
    ]


def build_wandb_interactive_charts(rows: list[dict[str, float]], wandb_module) -> dict[str, object]:
    xs = [row["time_s"] for row in rows]
    charts: dict[str, object] = {}
    for chart_id, title, _description, keys, _ylabel in get_plot_defs():
        ys = [series(rows, key) for key in keys]
        charts[f"interactive/{chart_id}"] = wandb_module.plot.line_series(
            xs=xs,
            ys=ys,
            keys=keys,
            title=title,
            xname="time_s",
        )
    return charts


def export_semantics(specs: list[ColumnSpec], output_dir: Path) -> None:
    csv_path = output_dir / "column_semantics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["source_column", "semantic_name", "group", "unit", "physical_meaning"])
        for spec in specs:
            writer.writerow([spec.source_column, spec.semantic_name, spec.group, spec.unit, spec.meaning])

    json_path = output_dir / "column_semantics.json"
    json_path.write_text(
        json.dumps([spec.__dict__ for spec in specs], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def export_summary(rows: list[dict[str, float]], specs: list[ColumnSpec], output_dir: Path) -> None:
    keys = [spec.semantic_name for spec in specs if spec.semantic_name != "step"]
    summary = summarize(rows, keys)
    (output_dir / "summary_stats.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def remap_rows(rows: list[dict[str, float]], specs: list[ColumnSpec], step_dt: float) -> list[dict[str, float]]:
    remapped: list[dict[str, float]] = []
    for row in rows:
        item: dict[str, float] = {}
        for spec in specs:
            item[spec.semantic_name] = row[spec.source_column]
        item["time_s"] = row["step"] * step_dt
        remapped.append(item)
    return remapped


def maybe_log_wandb(
    rows: list[dict[str, float]],
    output_dir: Path,
    summary_path: Path,
    semantics_path: Path,
    report_path: Path,
    project: str | None,
    run_name: str,
) -> bool:
    if not project:
        return False

    import wandb

    os.environ.setdefault("WANDB_MODE", "offline")
    wandb_root = output_dir / "wandb_offline"
    wandb_root.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("WANDB_DIR", str(wandb_root))
    os.environ.setdefault("WANDB_CACHE_DIR", str(wandb_root / "cache"))
    os.environ.setdefault("WANDB_CONFIG_DIR", str(wandb_root / "config"))

    settings = wandb.Settings(
        mode="offline",
        root_dir=str(wandb_root),
        start_method="thread",
        x_disable_stats=True,
        x_disable_meta=True,
    )
    try:
        run = wandb.init(
            project=project,
            name=run_name,
            job_type="b2-debug-analysis",
            settings=settings,
        )
    except Exception as exc:
        print("W&B offline logging was skipped because wandb could not start its local service.")
        print(f"Reason: {exc}")
        print("This usually works on a normal local machine even if it is blocked in a restricted sandbox.")
        return False
    if run is None:
        return

    artifact = wandb.Artifact(name=f"{run_name}-analysis", type="b2-debug-report")
    artifact.add_file(str(report_path))
    artifact.add_file(str(summary_path))
    artifact.add_file(str(semantics_path))

    svg_paths = sorted(output_dir.glob("*.svg"))
    for svg_path in svg_paths:
        artifact.add_file(str(svg_path))

    table = wandb.Table(columns=["source_column", "semantic_name", "group", "unit", "physical_meaning"])
    with semantics_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            table.add_data(
                row["source_column"],
                row["semantic_name"],
                row["group"],
                row["unit"],
                row["physical_meaning"],
            )
    log_payload = {"column_semantics": table}
    log_payload.update(build_wandb_interactive_charts(rows, wandb))

    # Keep the rendered report mirrored in the run page for quick reference.
    report_html = report_path.read_text(encoding="utf-8")
    log_payload["offline_report_html"] = wandb.Html(report_html, inject=False)

    wandb.log(log_payload)
    run.log_artifact(artifact)
    run.finish()
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze B2 velocity debug CSV and generate offline plots.")
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("deploy/robots/b2/log/b2_velocity_debug.csv"),
        help="Input CSV path.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("deploy/robots/b2/log/b2_velocity_debug_analysis"),
        help="Directory for HTML/SVG/summary outputs.",
    )
    parser.add_argument(
        "--step-dt",
        type=float,
        default=0.02,
        help="Control timestep in seconds used to convert step index to time.",
    )
    parser.add_argument(
        "--wandb-project",
        type=str,
        default=None,
        help="If set, create a W&B offline run and artifact under this project.",
    )
    parser.add_argument(
        "--wandb-run-name",
        type=str,
        default="b2-velocity-debug",
        help="W&B run name when --wandb-project is enabled.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    header, raw_rows = load_csv(args.input)
    specs = infer_schema(header)
    rows = remap_rows(raw_rows, specs, args.step_dt)

    export_semantics(specs, args.output_dir)
    export_summary(rows, specs, args.output_dir)
    build_report(rows, specs, args.output_dir, args.step_dt, args.input)

    wandb_logged = maybe_log_wandb(
        rows,
        args.output_dir,
        args.output_dir / "summary_stats.json",
        args.output_dir / "column_semantics.csv",
        args.output_dir / "report.html",
        args.wandb_project,
        args.wandb_run_name,
    )

    print(f"Analysis written to: {args.output_dir}")
    print(f"Open report: {args.output_dir / 'report.html'}")
    if args.wandb_project and wandb_logged:
        print("W&B offline run created. You can sync it later with:")
        print("  wandb sync wandb/offline-run-*")


if __name__ == "__main__":
    main()
