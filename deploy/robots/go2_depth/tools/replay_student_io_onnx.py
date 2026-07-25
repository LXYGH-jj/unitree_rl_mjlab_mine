"""Replay dumped play-side depth-student IO through a deployment ONNX model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


DEFAULT_ONNX = (
  Path(__file__).resolve().parents[1]
  / "config"
  / "policy"
  / "depth_student"
  / "v0"
  / "exported"
  / "policy.onnx"
)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--dump", required=True, help="Input .npz from dump_student_play_io.py.")
  parser.add_argument("--onnx", default=str(DEFAULT_ONNX), help="Deployment policy.onnx path.")
  parser.add_argument(
    "--mode",
    choices=("recorded-hidden", "roll-hidden", "both"),
    default="both",
    help="Use recorded play hidden_in, ONNX-rolled hidden, or compare both.",
  )
  parser.add_argument("--atol", type=float, default=1.0e-3, help="Warning threshold.")
  parser.add_argument("--max-steps", type=int, default=None, help="Limit replayed steps.")
  parser.add_argument("--print-every", type=int, default=20, help="Step print interval.")
  return parser.parse_args()


def _load_onnxruntime():
  try:
    import onnxruntime as ort
  except ImportError as exc:
    raise SystemExit("onnxruntime is required. Run this with the astro_parkour uv env.") from exc
  return ort


def _max_mean_abs(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
  diff = np.abs(a.astype(np.float32) - b.astype(np.float32))
  return float(diff.max()), float(diff.mean())


def _summarize(name: str, values: list[float]) -> str:
  if not values:
    return f"{name}: n=0"
  arr = np.asarray(values, dtype=np.float32)
  return (
    f"{name}: n={arr.size} max={float(arr.max()):.6g} "
    f"mean={float(arr.mean()):.6g} p95={float(np.percentile(arr, 95)):.6g}"
  )


def _run_recorded_hidden(
  session,
  obs: np.ndarray,
  depth: np.ndarray,
  hidden_in: np.ndarray,
  ref_actions: np.ndarray,
  ref_hidden_out: np.ndarray,
  print_every: int,
) -> tuple[list[float], list[float]]:
  action_max_values: list[float] = []
  hidden_max_values: list[float] = []
  for step in range(obs.shape[0]):
    actions, hidden_out = session.run(
      None,
      {
        "obs": obs[step],
        "depth": depth[step],
        "hidden_in": hidden_in[step],
      },
    )
    action_max, action_mean = _max_mean_abs(actions, ref_actions[step])
    hidden_max, hidden_mean = _max_mean_abs(hidden_out, ref_hidden_out[step])
    action_max_values.append(action_max)
    hidden_max_values.append(hidden_max)
    if step % print_every == 0 or step == obs.shape[0] - 1:
      print(
        f"[recorded-hidden {step:04d}] "
        f"action_max={action_max:.6g} action_mean={action_mean:.6g} "
        f"hidden_max={hidden_max:.6g} hidden_mean={hidden_mean:.6g}"
      )
  return action_max_values, hidden_max_values


def _run_roll_hidden(
  session,
  obs: np.ndarray,
  depth: np.ndarray,
  hidden_in: np.ndarray,
  ref_actions: np.ndarray,
  ref_hidden_out: np.ndarray,
  dones: np.ndarray,
  print_every: int,
) -> tuple[list[float], list[float]]:
  action_max_values: list[float] = []
  hidden_max_values: list[float] = []
  hidden = hidden_in[0].copy()
  for step in range(obs.shape[0]):
    actions, hidden_out = session.run(
      None,
      {
        "obs": obs[step],
        "depth": depth[step],
        "hidden_in": hidden,
      },
    )
    action_max, action_mean = _max_mean_abs(actions, ref_actions[step])
    hidden_max, hidden_mean = _max_mean_abs(hidden_out, ref_hidden_out[step])
    action_max_values.append(action_max)
    hidden_max_values.append(hidden_max)
    if step % print_every == 0 or step == obs.shape[0] - 1:
      print(
        f"[roll-hidden     {step:04d}] "
        f"action_max={action_max:.6g} action_mean={action_mean:.6g} "
        f"hidden_max={hidden_max:.6g} hidden_mean={hidden_mean:.6g}"
      )
    hidden = hidden_out.astype(np.float32)
    done_mask = dones[step].astype(bool)
    if np.any(done_mask):
      hidden[:, done_mask, :] = 0.0
  return action_max_values, hidden_max_values


def main() -> None:
  args = parse_args()
  ort = _load_onnxruntime()

  dump_path = Path(args.dump)
  onnx_path = Path(args.onnx)
  data = np.load(dump_path)
  obs = data["obs"].astype(np.float32)
  depth = data["depth"].astype(np.float32)
  hidden_in = data["hidden_in"].astype(np.float32)
  ref_actions = data["actions"].astype(np.float32)
  ref_hidden_out = data["hidden_out"].astype(np.float32)
  dones = data["dones"].astype(bool)
  if args.max_steps is not None:
    n = min(args.max_steps, obs.shape[0])
    obs = obs[:n]
    depth = depth[:n]
    hidden_in = hidden_in[:n]
    ref_actions = ref_actions[:n]
    ref_hidden_out = ref_hidden_out[:n]
    dones = dones[:n]

  print(f"[INFO] dump: {dump_path}")
  print(f"[INFO] onnx: {onnx_path}")
  print(
    f"[INFO] shapes obs={obs.shape} depth={depth.shape} "
    f"hidden={hidden_in.shape} actions={ref_actions.shape}"
  )

  session = ort.InferenceSession(str(onnx_path.resolve()))
  input_names = [inp.name for inp in session.get_inputs()]
  output_names = [out.name for out in session.get_outputs()]
  print(f"[INFO] onnx inputs={input_names} outputs={output_names}")

  failed = False
  if args.mode in ("recorded-hidden", "both"):
    action_values, hidden_values = _run_recorded_hidden(
      session,
      obs,
      depth,
      hidden_in,
      ref_actions,
      ref_hidden_out,
      args.print_every,
    )
    print("[RESULT] " + _summarize("recorded-hidden action_max_abs", action_values))
    print("[RESULT] " + _summarize("recorded-hidden hidden_max_abs", hidden_values))
    failed = failed or max(action_values, default=0.0) > args.atol

  if args.mode in ("roll-hidden", "both"):
    action_values, hidden_values = _run_roll_hidden(
      session,
      obs,
      depth,
      hidden_in,
      ref_actions,
      ref_hidden_out,
      dones,
      args.print_every,
    )
    print("[RESULT] " + _summarize("roll-hidden action_max_abs", action_values))
    print("[RESULT] " + _summarize("roll-hidden hidden_max_abs", hidden_values))
    failed = failed or max(action_values, default=0.0) > args.atol

  if failed:
    raise SystemExit(f"[WARN] action max_abs exceeded atol={args.atol:.6g}")
  print("[INFO] replay comparison passed.")


if __name__ == "__main__":
  main()
