#!/usr/bin/env python3
# train_rl_residual.py
# Stage 3 — TD3+BC Offline Reinforcement Learning
#
# Trains a residual policy using TD3+BC (Fujimoto & Gu, 2021) on the MPPI episode dataset.
# The actor learns to maximize long-term cumulative CTE reduction instead of just
# imitating heuristic actions from the dataset (Behavioral Cloning).
# This allows it to surpass the base MPPI performance through the reward signal.
#
# Architecture:
#   Actor  (policy) : state (20D) → action ∈ [-1,1]² [exported to TorchScript]
#   Critic (twin Q) : (state, action) → Q scalar     [training only]
#
# Inference contract with controller_rl_residual.py remains identical:
#   - Same .ts file (TorchScript from Actor)
#   - Same norm_mean/norm_std keys in the .pt payload
#   - Same input/output dimensions (20D → 2D)
#
# Usage:
#   ros2 run vrx_experiment_benchmark train_rl_residual --ros-args \
#     -p dataset_root:=.../metrics/rl_dataset/raw \
#     -p output_model_dir:=.../config/rl_models \
#     -p n_gradient_steps:=100000 \
#     -p train_device:=auto

import csv
import copy
import glob
import math
import os
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node


# ──────────────────────────────────────────────────────────────────────────────
# Episode Metadata
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class EpisodeMeta:
    path:       str
    route_name: str
    stage:      str
    timestamp:  str

    @property
    def key(self) -> str:
        return f"{self.route_name}__{self.stage}"


# ──────────────────────────────────────────────────────────────────────────────
# Main Node
# ──────────────────────────────────────────────────────────────────────────────

class TrainRLResidual(Node):

    def __init__(self) -> None:
        super().__init__("train_rl_residual")

        # Directories
        self.declare_parameter("dataset_root",            "metrics/rl_dataset/raw")
        self.declare_parameter("output_model_dir",        "config/rl_models")
        self.declare_parameter("output_model_name",       "mppi_rl_residual_policy.pt")
        self.declare_parameter("output_torchscript_name", "mppi_rl_residual_policy.ts")
        self.declare_parameter("split_manifest_dir",      "metrics/rl_dataset/splits")
        self.declare_parameter("training_log_dir",        "metrics/rl_dataset/training")
        self.declare_parameter("run_training_immediately", True)

        # Split configuration
        self.declare_parameter("random_seed",                  42)
        self.declare_parameter("validation_episodes_per_key",  2)
        self.declare_parameter("min_rows_per_episode",         50)
        self.declare_parameter("exclude_stage1",               True)

        # Architecture parameters
        self.declare_parameter("policy_input_dim",   20)
        self.declare_parameter("policy_hidden_dim",  128)   # Actor hidden
        self.declare_parameter("critic_hidden_dim",  256)   # Critic hidden (larger)

        # TD3+BC hyperparameters
        # n_gradient_steps increased: BC converges ~14K, more steps for RL exploration
        self.declare_parameter("n_gradient_steps",  200_000)
        self.declare_parameter("batch_size",         256)
        # lr_actor increased: actor needs to respond to RL gradient
        self.declare_parameter("lr_actor",           2e-4)
        self.declare_parameter("lr_critic",          3e-4)   # 1e-3 caused spikes; 3e-4 is more stable
        self.declare_parameter("weight_decay",       1e-4)
        # gamma 0.99: effective horizon ~100 steps (≈5s at 20Hz), captures cumulative benefits
        self.declare_parameter("gamma",              0.99)
        self.declare_parameter("tau",                0.005)   # Soft-update rate
        # td3bc_alpha 3.0: with |Q|≈8 and alpha=3.0 → λ≈0.37, RL gradient has 2x weight
        # (previously alpha=1.5 → λ≈0.25 and -λQ cancelled with BC loss leaving actor stuck)
        self.declare_parameter("td3bc_alpha",        3.0)
        self.declare_parameter("gradient_clip_norm", 1.0)
        # val_every_n_steps reduced: less validation overhead for more RL steps
        self.declare_parameter("val_every_n_steps",  5000)
        self.declare_parameter("train_device",       "auto")

        # Reward design
        # reward_scale 22: with quadratic CTE, Q_real≈−231 unscaled.
        #   Q_scaled = Q_real / scale = −231/22 ≈ −10.5  →  λ = 3.0/10.5 ≈ 0.29  ✓
        # Derived from training log: mean_r_real ≈ −2.31, gamma=0.99
        #   Q_real = −2.31/0.01 = −231; correct scale = 231/10.5 ≈ 22
        self.declare_parameter("reward_scale",           22.0)
        # Quadratic penalty: (CTE / cte_norm)² — gradient proportional to error
        self.declare_parameter("reward_cte_weight",     1.0)
        self.declare_parameter("reward_cte_norm",       2.0)   # normalization in meters; (CTE/2.0)²
        # CTE improvement bonus (dense signal for active reduction), increased
        self.declare_parameter("reward_improve_weight", 0.5)
        # Speed bonus: encourages keeping speed ≥ 80% u_ref without sacrificing CTE
        self.declare_parameter("reward_speed_weight",   0.15)
        # Action penalty reduced: physical clamped residual ensures action bounds
        self.declare_parameter("reward_action_weight",  0.05)

        # Residual limits (for action normalization)
        self.declare_parameter("max_residual_common_transit", 120.0)
        self.declare_parameter("max_residual_diff_transit",   150.0)
        self.declare_parameter("max_residual_common_hold",     60.0)
        self.declare_parameter("max_residual_diff_hold",       80.0)

        # Read parameters
        g = self.get_parameter
        self.dataset_root            = str(g("dataset_root").value)
        self.output_model_dir        = str(g("output_model_dir").value)
        self.output_model_name       = str(g("output_model_name").value)
        self.output_torchscript_name = str(g("output_torchscript_name").value)
        self.split_manifest_dir      = str(g("split_manifest_dir").value)
        self.training_log_dir        = str(g("training_log_dir").value)
        self.run_training_immediately = bool(g("run_training_immediately").value)

        self.random_seed          = int(g("random_seed").value)
        self.val_per_key          = int(g("validation_episodes_per_key").value)
        self.min_rows_per_episode = int(g("min_rows_per_episode").value)
        self.exclude_stage1       = bool(g("exclude_stage1").value)

        self.policy_input_dim  = int(g("policy_input_dim").value)
        self.policy_hidden_dim = int(g("policy_hidden_dim").value)
        self.critic_hidden_dim = int(g("critic_hidden_dim").value)

        self.n_gradient_steps  = int(g("n_gradient_steps").value)
        self.batch_size        = int(g("batch_size").value)
        self.lr_actor          = float(g("lr_actor").value)
        self.lr_critic         = float(g("lr_critic").value)
        self.weight_decay      = float(g("weight_decay").value)
        self.gamma             = float(g("gamma").value)
        self.tau               = float(g("tau").value)
        self.td3bc_alpha       = float(g("td3bc_alpha").value)
        self.gradient_clip_norm = float(g("gradient_clip_norm").value)
        self.val_every_n_steps = int(g("val_every_n_steps").value)
        self.train_device      = str(g("train_device").value)

        self.reward_scale           = float(g("reward_scale").value)
        self.reward_cte_weight     = float(g("reward_cte_weight").value)
        self.reward_cte_norm       = float(g("reward_cte_norm").value)
        self.reward_improve_weight = float(g("reward_improve_weight").value)
        self.reward_speed_weight   = float(g("reward_speed_weight").value)
        self.reward_action_weight  = float(g("reward_action_weight").value)

        self.max_rc_transit = float(g("max_residual_common_transit").value)
        self.max_rd_transit = float(g("max_residual_diff_transit").value)
        self.max_rc_hold    = float(g("max_residual_common_hold").value)
        self.max_rd_hold    = float(g("max_residual_diff_hold").value)

        self._rng = random.Random(self.random_seed)

        if self.run_training_immediately:
            self.train_td3bc()

    # ══════════════════════════════════════════════════════════════════════════
    # Directory Utilities
    # ══════════════════════════════════════════════════════════════════════════

    def _resolve_dir(self, path: str) -> str:
        for candidate in [os.path.abspath(path)]:
            try:
                os.makedirs(candidate, exist_ok=True)
                probe = os.path.join(candidate, ".probe")
                open(probe, "w").close()
                os.remove(probe)
                return candidate
            except OSError:
                pass
        fb = os.path.join("/tmp", "vrx_rl_training")
        os.makedirs(fb, exist_ok=True)
        self.get_logger().warn(f"[train] fallback dir: {fb}")
        return fb

    # ══════════════════════════════════════════════════════════════════════════
    # Episode Parsing
    # ══════════════════════════════════════════════════════════════════════════

    KNOWN_ROUTES = [
        "route_straight.yaml",
        "route_curves.yaml",
        "route_zigzag.yaml",
    ]

    def _detect_stage_from_csv(self, path: str) -> str:
        try:
            with open(path, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for i, row in enumerate(reader):
                    if i > 30:
                        break
                    sm = row.get("shadow_mode", "").strip()
                    dc = row.get("delta_common", "0").strip()
                    try:
                        if sm in ("True", "1", "true"):
                            return "stage1"
                        if float(dc) != 0.0:
                            return "stage2"
                    except ValueError:
                        pass
        except Exception:
            pass
        return "stage2"

    def _parse_episode(self, path: str) -> Optional[EpisodeMeta]:
        name = os.path.basename(path)
        if not name.endswith("_dataset.csv"):
            return None
        route = next((r for r in self.KNOWN_ROUTES if name.startswith(r)), None)
        if route is None:
            return None
        ts_part = name[len(route):].lstrip("_").replace("_dataset.csv", "")
        stage = self._detect_stage_from_csv(path)
        return EpisodeMeta(path=path, route_name=route, stage=stage, timestamp=ts_part)

    # ══════════════════════════════════════════════════════════════════════════
    # Discovery and Split
    # ══════════════════════════════════════════════════════════════════════════

    def discover_episodes(self) -> List[EpisodeMeta]:
        pattern = os.path.join(self.dataset_root, "*.csv")
        files = sorted(glob.glob(pattern))
        episodes: List[EpisodeMeta] = []
        skipped_short = 0
        skipped_stage1 = 0

        for path in files:
            meta = self._parse_episode(path)
            if meta is None:
                continue
            if self.exclude_stage1 and meta.stage == "stage1":
                skipped_stage1 += 1
                continue
            with open(path, "r", encoding="utf-8") as f:
                n_rows = sum(1 for _ in f) - 1
            if n_rows < self.min_rows_per_episode:
                skipped_short += 1
                continue
            episodes.append(meta)

        if not episodes:
            raise RuntimeError(
                f"No se encontraron episodios útiles en {self.dataset_root}\n"
                f"  stage1 ignorados: {skipped_stage1} | cortos: {skipped_short}"
            )

        by_key: Dict[str, int] = {}
        for ep in episodes:
            by_key[ep.key] = by_key.get(ep.key, 0) + 1

        self.get_logger().info(
            f"[train] {len(episodes)} episodes discovered "
            f"(excluded stage1={skipped_stage1}, short={skipped_short})"
        )
        for key, cnt in sorted(by_key.items()):
            self.get_logger().info(f"[train]   {key}: {cnt} episodes")

        return episodes

    def make_stratified_split(
        self, episodes: List[EpisodeMeta]
    ) -> Tuple[List[EpisodeMeta], List[EpisodeMeta]]:
        by_key: Dict[str, List[EpisodeMeta]] = {}
        for ep in episodes:
            by_key.setdefault(ep.key, []).append(ep)

        train_eps: List[EpisodeMeta] = []
        val_eps:   List[EpisodeMeta] = []

        for key in sorted(by_key.keys()):
            bucket = list(by_key[key])
            bucket.sort(key=lambda e: e.timestamp)
            self._rng.shuffle(bucket)
            n_val = min(self.val_per_key, max(1, len(bucket) // 5))
            val_eps.extend(bucket[:n_val])
            train_eps.extend(bucket[n_val:])

        self.get_logger().info(f"[train] Split | train={len(train_eps)} val={len(val_eps)}")
        return train_eps, val_eps

    def write_split_manifests(
        self, train_eps: List[EpisodeMeta], val_eps: List[EpisodeMeta]
    ) -> None:
        out_dir = self._resolve_dir(self.split_manifest_dir)
        header = ["route_name", "stage", "timestamp", "path"]
        for subset, name in [
            (train_eps, "train_manifest.csv"),
            (val_eps,   "val_manifest.csv"),
        ]:
            fpath = os.path.join(out_dir, name)
            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(header)
                for ep in sorted(subset, key=lambda e: (e.route_name, e.stage, e.timestamp)):
                    w.writerow([ep.route_name, ep.stage, ep.timestamp, ep.path])
        self.get_logger().info(f"[train] Manifests written to {out_dir}")

    # ══════════════════════════════════════════════════════════════════════════
    # Feature Extraction and Transition Building
    # ══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _residual_limits(self, mode_code: int) -> Tuple[float, float]:
        if mode_code in (0, 2):
            return self.max_rc_hold, self.max_rd_hold
        return self.max_rc_transit, self.max_rd_transit

    def _row_to_features(self, row: dict) -> Optional[Tuple]:
        """Extracts (feat_20d, mode_code, e_ct, delta_common, delta_diff) from a CSV row.

        Returns None if row is incomplete or contains NaN. Feature vector is
        identical to the one built by controller_rl_residual.py in build_observation(),
        ensuring train/inference coherence.
        """
        try:
            yaw          = float(row["yaw"])
            psi_ref      = float(row["psi_ref"])
            vx           = float(row["vx"])
            vy           = float(row["vy"])
            speed        = float(row["speed"])
            yaw_rate     = float(row["yaw_rate"])
            e_ct         = float(row["e_ct"])
            heading_error = float(row["heading_error"])
            u_ref        = float(row["u_ref"])
            dist_to_wp   = float(row["dist_to_wp"])
            mode_code    = int(float(row["mode_code"]))
            pos_tol      = float(row["pos_tol"])
            common_raw   = float(row["common_raw"])
            diff_raw     = float(row["diff_raw"])
            chi_p        = float(row.get("chi_p", row.get("chi", str(psi_ref))))
            delta_common = float(row["delta_common"])
            delta_diff   = float(row["delta_diff"])
        except (KeyError, ValueError):
            return None

        # Detect NaN/Inf
        vals = [yaw, psi_ref, vx, vy, speed, yaw_rate, e_ct, heading_error,
                u_ref, dist_to_wp, pos_tol, common_raw, diff_raw, chi_p,
                delta_common, delta_diff]
        if any(not math.isfinite(v) for v in vals):
            return None

        speed_error = u_ref - speed
        x = [
            math.sin(yaw),  math.cos(yaw),           # 0-1 vehicle orientation
            math.sin(psi_ref), math.cos(psi_ref),    # 2-3 reference LOS heading
            vx, vy, speed, yaw_rate,                 # 4-7 velocity components
            e_ct, heading_error,                     # 8-9 lateral and heading error
            self._wrap(psi_ref - yaw),               # 10 yaw error
            u_ref, speed_error, dist_to_wp,          # 11-13 references
            float(mode_code), pos_tol,               # 14-15 mode and tolerance
            common_raw / 1000.0,                     # 16 base common command
            diff_raw   / 1000.0,                     # 17 base differential command
            math.sin(chi_p),                         # 18 path geometry
            math.cos(chi_p),                         # 19
        ]
        return x, mode_code, e_ct, delta_common, delta_diff

    def _episode_to_transitions(self, path: str) -> List[Tuple]:
        """Reads a CSV episode and builds transitions (s, a, r, s', done).

        Reward Design (v2 — quadratic + speed bonus):

          r_t = - w_cte  × (CTE_{t+1} / cte_norm)²         ← quadratic penalty
                + w_imp  × max(0, |CTE_t| − |CTE_{t+1}|)   ← active reduction bonus
                + w_spd  × max(0, speed_{t+1} − 0.8×u_ref) ← speed bonus
                − w_act  × (a₀² + a₁²)                     ← action penalty

        Advantages over linear penalty:
        - Quadratic CTE: gradient ∝ CTE, pushes harder for larger errors and relaxes
          near zero, avoiding high-frequency oscillations.
        - Speed bonus: explicitly encourages speed without sacrificing CTE.
          Only rewards if speed ≥ 80% of u_ref (conservative threshold).
        - Reduced w_act: physical clamp already limits residuals; less weight here
          gives actor more freedom to act when necessary.

        Note: done=1 only on the last row of the episode.
        """
        rows_raw: List[dict] = []
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows_raw.append(dict(row))

        parsed: List[Optional[Tuple]] = [self._row_to_features(r) for r in rows_raw]

        transitions: List[Tuple] = []
        w_cte  = self.reward_cte_weight
        cte_norm = max(self.reward_cte_norm, 0.1)  # m de normalización del CTE
        w_imp  = self.reward_improve_weight
        w_spd  = self.reward_speed_weight
        w_act  = self.reward_action_weight

        for i in range(len(parsed)):
            cur = parsed[i]
            if cur is None:
                continue
            x, mode_code, e_ct, delta_c, delta_d = cur

            # Filter rows where residual was zero (stage1 or inactive)
            if abs(delta_c) < 1e-4 and abs(delta_d) < 1e-4:
                continue

            # Normalize action ∈ [-1, 1]
            lim_c, lim_d = self._residual_limits(mode_code)
            a = [
                max(-1.0, min(1.0, delta_c / max(lim_c, 1e-6))),
                max(-1.0, min(1.0, delta_d / max(lim_d, 1e-6))),
            ]

            # Next state and reward
            if i + 1 < len(parsed) and parsed[i + 1] is not None:
                x_next, _, e_ct_next, _, _ = parsed[i + 1]

                # Get next speed and u_ref to compute speed bonus
                # x[6]=speed, x[11]=u_ref  (see _row_to_features)
                speed_next = x_next[6]
                u_ref_next = x_next[11]

                # Quadratic CTE Penalty (normalized)
                cte_penalty = w_cte * (e_ct_next / cte_norm) ** 2

                # Active CTE Reduction Bonus
                improve_bonus = w_imp * max(0.0, abs(e_ct) - abs(e_ct_next))

                # Speed Bonus: rewards speed ≥ 80% of u_ref
                # Only active in transit mode (mode_code == 1) to avoid interfering
                # with waypoint capture phases (start/finish).
                speed_bonus = 0.0
                if mode_code == 1:
                    speed_bonus = w_spd * max(0.0, speed_next - 0.80 * u_ref_next)

                # Action Penalty (anti-saturation)
                action_pen = w_act * (a[0] ** 2 + a[1] ** 2)

                r    = -(cte_penalty - improve_bonus - speed_bonus + action_pen)
                done = 0.0
            else:
                # Last step of episode: bootstrap with current state
                x_next = x
                cte_penalty = w_cte * (e_ct / cte_norm) ** 2
                action_pen  = w_act * (a[0] ** 2 + a[1] ** 2)
                r    = -(cte_penalty + action_pen)
                done = 1.0

            transitions.append((x, a, r, x_next, done))

        return transitions

    def build_replay_buffer(
        self, episodes: List[EpisodeMeta]
    ) -> Tuple[List, List, List, List, List]:
        """Builds replay buffer and normalizes rewards.

        Reward Normalization (critical for TD3+BC):
          r_raw = -w_cte*|CTE_next| + w_improve*max(0, |CTE|-|CTE_next|)
          r_scaled = r_raw / reward_scale

        With reward_scale≈10 and avg CTE ≈2m:
          r_scaled ≈ -0.2  →  Q ≈ -0.2/0.03 ≈ -6.7
          λ = 2.5/6.7 ≈ 0.37  (useful range: 0.2-0.6)

        This prevents λ collapse seen with unscaled rewards (λ≈0.04).
        """
        S, A, R, S_next, D = [], [], [], [], []
        for ep in episodes:
            for s, a, r, s_next, done in self._episode_to_transitions(ep.path):
                S.append(s)
                A.append(a)
                R.append([r / max(self.reward_scale, 1e-6)])
                S_next.append(s_next)
                D.append([done])

        if not S:
            raise RuntimeError(
                "Empty replay buffer. Verify episodes contain non-zero residuals."
            )

        # Reward stats logging
        r_flat = [row[0] for row in R]
        r_min  = min(r_flat)
        r_max  = max(r_flat)
        r_mean = sum(r_flat) / max(len(r_flat), 1)
        r_std  = math.sqrt(
            sum((v - r_mean) ** 2 for v in r_flat) / max(len(r_flat) - 1, 1)
        )
        q_est  = r_mean / max(1.0 - self.gamma, 1e-6)
        lam_est = self.td3bc_alpha / max(abs(q_est), 1e-6)
        self.get_logger().info(
            f"[train] Scaled Reward | "
            f"min={r_min:.4f} max={r_max:.4f} mean={r_mean:.4f} std={r_std:.4f}\n"
            f"         Est Q ≈ {q_est:.2f}  →  Est λ ≈ {lam_est:.3f} "
            f"(target: 0.2-1.0)"
        )

        return S, A, R, S_next, D

    # ══════════════════════════════════════════════════════════════════════════
    # TD3+BC Pipeline
    # ══════════════════════════════════════════════════════════════════════════

    def train_td3bc(self) -> None:  # noqa: C901
        try:
            import torch
            import torch.nn as nn
            import torch.nn.functional as F
        except ImportError as exc:
            raise RuntimeError(f"PyTorch required for Stage 3: {exc}") from exc

        # Network Definitions

        state_dim  = self.policy_input_dim   # 20
        action_dim = 2

        class Actor(nn.Module):
            """Policy: state -> action in [-1, 1]².
            BatchNorm1d for training stability; functions with running stats at eval.
            Architecture identical to BC version for inference compatibility.
            """
            def __init__(self, s_dim: int, hid: int):
                super().__init__()
                self.net = nn.Sequential(
                    nn.Linear(s_dim, hid), nn.BatchNorm1d(hid), nn.Tanh(),
                    nn.Linear(hid, hid),   nn.BatchNorm1d(hid), nn.Tanh(),
                    nn.Linear(hid, 2),     nn.Tanh(),
                )

            def forward(self, x: torch.Tensor) -> torch.Tensor:
                return self.net(x)

        class Critic(nn.Module):
            """Twin Q-networks: (s, a) -> (Q1, Q2).
            No BatchNorm in critics—Q scaling interacts poorly with BN
            during TD training.
            """
            def __init__(self, s_dim: int, a_dim: int, hid: int):
                super().__init__()
                sa_dim = s_dim + a_dim

                def _q() -> nn.Sequential:
                    return nn.Sequential(
                        nn.Linear(sa_dim, hid), nn.ReLU(),
                        nn.Linear(hid, hid),    nn.ReLU(),
                        nn.Linear(hid, 1),
                    )

                self.q1 = _q()
                self.q2 = _q()

            def forward(
                self, s: torch.Tensor, a: torch.Tensor
            ) -> Tuple[torch.Tensor, torch.Tensor]:
                sa = torch.cat([s, a], dim=1)
                return self.q1(sa), self.q2(sa)

        def soft_update(target: nn.Module, src: nn.Module, tau: float) -> None:
            for tp, sp in zip(target.parameters(), src.parameters()):
                tp.data.mul_(1.0 - tau).add_(tau * sp.data)

        # Discover datasets
        episodes = self.discover_episodes()
        train_eps, val_eps = self.make_stratified_split(episodes)
        self.write_split_manifests(train_eps, val_eps)

        # Build replay buffers
        self.get_logger().info("[train] Building training replay buffer...")
        S_tr, A_tr, R_tr, SN_tr, D_tr = self.build_replay_buffer(train_eps)
        self.get_logger().info("[train] Building validation replay buffer...")
        S_va, A_va, _,    _,     _    = self.build_replay_buffer(val_eps)

        N_train, N_val = len(S_tr), len(S_va)
        self.get_logger().info(f"[train] Transitions | train={N_train} val={N_val}")

        # Convert to tensors
        S_tr  = torch.tensor(S_tr,  dtype=torch.float32)
        A_tr  = torch.tensor(A_tr,  dtype=torch.float32)
        R_tr  = torch.tensor(R_tr,  dtype=torch.float32)
        SN_tr = torch.tensor(SN_tr, dtype=torch.float32)
        D_tr  = torch.tensor(D_tr,  dtype=torch.float32)
        S_va  = torch.tensor(S_va,  dtype=torch.float32)
        A_va  = torch.tensor(A_va,  dtype=torch.float32)

        # State Normalization (train mean/std)
        s_mean = S_tr.mean(dim=0)
        s_std  = S_tr.std(dim=0).clamp(min=1e-6)

        S_tr_n  = (S_tr  - s_mean) / s_std
        SN_tr_n = (SN_tr - s_mean) / s_std
        S_va_n  = (S_va  - s_mean) / s_std

        # Compute Device Selection
        raw_device = self.train_device
        if raw_device == "auto":
            raw_device = "cuda" if torch.cuda.is_available() else "cpu"
        device = torch.device(raw_device)

        if raw_device == "cuda":
            self.get_logger().info(
                f"[train] Device: CUDA ({torch.cuda.get_device_name(0)})"
            )
        else:
            self.get_logger().info("[train] Device: CPU")

        # Move inputs to device (130K x 20D x 4B ≈ 10MB per tensor — easily fits in VRAM)
        S_tr_n  = S_tr_n.to(device)
        A_tr    = A_tr.to(device)
        R_tr    = R_tr.to(device)
        SN_tr_n = SN_tr_n.to(device)
        D_tr    = D_tr.to(device)
        S_va_n  = S_va_n.to(device)
        A_va    = A_va.to(device)

        # Network instantiation
        actor  = Actor(state_dim, self.policy_hidden_dim).to(device)
        critic = Critic(state_dim, action_dim, self.critic_hidden_dim).to(device)

        actor_target  = copy.deepcopy(actor).to(device)
        critic_target = copy.deepcopy(critic).to(device)
        actor_target.eval()
        critic_target.eval()

        opt_actor  = torch.optim.Adam(
            actor.parameters(),  lr=self.lr_actor,  weight_decay=self.weight_decay
        )
        opt_critic = torch.optim.Adam(
            critic.parameters(), lr=self.lr_critic, weight_decay=self.weight_decay
        )

        # BC Warm Start (initialize actor prior to Q updates)
        # 3000 steps ensures the critic has decent Q-estimates before the actor
        # begins optimizing against them, reducing early divergence.
        WARMUP_STEPS = 3000
        self.get_logger().info(f"[train] Warm-start BC ({WARMUP_STEPS} pasos)...")
        actor.train()
        for ws in range(WARMUP_STEPS):
            idx = torch.randint(0, N_train, (self.batch_size,), device=device)
            actor_loss_bc = F.mse_loss(actor(S_tr_n[idx]), A_tr[idx])
            opt_actor.zero_grad()
            actor_loss_bc.backward()
            opt_actor.step()

        # Propagate warm-start to target networks
        actor_target = copy.deepcopy(actor)
        actor_target.eval()
        bc_loss_warmup = float(actor_loss_bc.item())
        self.get_logger().info(
            f"[train] Warm-start complete | bc_loss_final={bc_loss_warmup:.5f}"
        )

        # Main TD3+BC Training Loop
        gamma = self.gamma
        tau   = self.tau
        alpha = self.td3bc_alpha

        best_val_bc   = float("inf")
        best_actor_sd = None
        history: List[Tuple] = []

        self.get_logger().info(
            f"[train] TD3+BC | steps={self.n_gradient_steps} "
            f"γ={gamma} τ={tau} α={alpha} "
            f"lr_a={self.lr_actor} lr_c={self.lr_critic}"
        )

        for step in range(1, self.n_gradient_steps + 1):
            idx  = torch.randint(0, N_train, (self.batch_size,), device=device)
            s_b  = S_tr_n[idx]
            a_b  = A_tr[idx]
            r_b  = R_tr[idx]
            sn_b = SN_tr_n[idx]
            d_b  = D_tr[idx]

            # 1. Update Critic (Bellman backup)
            with torch.no_grad():
                actor_target.eval()
                a_next       = actor_target(sn_b)
                q1_t, q2_t  = critic_target(sn_b, a_next)
                q_target     = r_b + gamma * torch.min(q1_t, q2_t) * (1.0 - d_b)

            critic.train()
            q1, q2      = critic(s_b, a_b)
            critic_loss = F.mse_loss(q1, q_target) + F.mse_loss(q2, q_target)

            opt_critic.zero_grad()
            critic_loss.backward()
            torch.nn.utils.clip_grad_norm_(critic.parameters(), self.gradient_clip_norm)
            opt_critic.step()

            # 2. Update Actor (TD3+BC)
            # lambda normalizes Q scale with respect to BC loss
            actor.train()
            a_pred   = actor(s_b)
            q1_pi, _ = critic(s_b, a_pred)
            lmbda    = alpha / (q1_pi.abs().mean().detach() + 1e-8)
            actor_loss = -lmbda * q1_pi.mean() + F.mse_loss(a_pred, a_b)

            opt_actor.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), self.gradient_clip_norm)
            opt_actor.step()

            # 3. Target Network Soft-Updates
            soft_update(actor_target,  actor,  tau)
            soft_update(critic_target, critic, tau)

            # 4. Periodic Validation
            if step % self.val_every_n_steps == 0:
                actor.eval()
                with torch.no_grad():
                    val_pred = actor(S_va_n)
                    val_bc   = float(F.mse_loss(val_pred, A_va).item())
                    c_l = float(critic_loss.item())
                    a_l = float(actor_loss.item())
                    lam = float(lmbda.item())

                    # val_cte_align_score
                    # Measures if actor shifts delta_diff in the same direction as
                    # e_ct (which is correct: e_ct>0 -> delta_diff>0 to correct).
                    # Heuristic MPPI computes: delta_diff = k_cte*tanh(e_ct) + ...
                    # -> sign(delta_diff) == sign(e_ct) when e_ct dominates.
                    # Score in [-1, 1]: positive means policy aligns with e_ct.
                    e_ct_val  = S_va_n[:, 8]           # índice 8 = e_ct normalizado
                    diff_pred = val_pred[:, 1]          # canal 1 = delta_diff
                    align = (e_ct_val.sign() * diff_pred.sign()).mean().item()

                history.append((step, c_l, a_l, val_bc, lam))

                if val_bc < best_val_bc:
                    best_val_bc   = val_bc
                    best_actor_sd = {k: v.cpu().clone()
                                     for k, v in actor.state_dict().items()}

                self.get_logger().info(
                    f"[train] step:{step:6d}/{self.n_gradient_steps} | "
                    f"critic:{c_l:.5f} actor:{a_l:.5f} λ:{lam:.4f} | "
                    f"val_bc:{val_bc:.5f} (best:{best_val_bc:.5f}) | "
                    f"cte_align:{align:.3f}"
                )

        # Export Trained Model
        if best_actor_sd is None:
            raise RuntimeError("Training failed: no model was saved.")

        actor.load_state_dict(best_actor_sd)
        actor.eval()

        out_dir    = self._resolve_dir(self.output_model_dir)
        model_path = os.path.join(out_dir, self.output_model_name)
        ts_path    = os.path.join(out_dir, self.output_torchscript_name)

        # .pt Payload — Same keys as BC version (controller contract)
        payload = {
            "state_dict":    best_actor_sd,
            "norm_mean":     s_mean.cpu(),
            "norm_std":      s_std.cpu(),
            "input_dim":     state_dim,
            "hidden_dim":    self.policy_hidden_dim,
            "best_val_loss": best_val_bc,   # BC alias for compatibility
            "algorithm":     "TD3+BC",
        }
        torch.save(payload, model_path)

        # TorchScript Actor (actor only — same inference contract)
        example  = torch.randn(1, state_dim, dtype=torch.float32).to(device)
        scripted = torch.jit.trace(actor, example)
        scripted.save(ts_path)

        self._write_training_history(history, best_val_bc)
        self.get_logger().info(
            f"[train] ✅ TD3+BC complete | best_val_bc={best_val_bc:.6f}\n"
            f"         model       → {model_path}\n"
            f"         torchscript → {ts_path}"
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Training History Logs
    # ══════════════════════════════════════════════════════════════════════════

    def _write_training_history(self, history: List[Tuple], best_val: float) -> None:
        out_dir  = self._resolve_dir(self.training_log_dir)
        csv_path = os.path.join(out_dir, "training_history.csv")
        txt_path = os.path.join(out_dir, "training_summary.txt")

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["step", "critic_loss", "actor_loss", "val_bc_loss", "lambda"])
            for row in history:
                w.writerow([row[0],
                             f"{row[1]:.8f}", f"{row[2]:.8f}",
                             f"{row[3]:.8f}", f"{row[4]:.6f}"])

        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("=== Etapa 3 — TD3+BC training summary ===\n")
            f.write(f"algorithm             = TD3+BC\n")
            f.write(f"n_gradient_steps      = {self.n_gradient_steps}\n")
            f.write(f"best_val_bc_loss      = {best_val:.8f}\n")
            f.write(f"gamma                 = {self.gamma}\n")
            f.write(f"tau                   = {self.tau}\n")
            f.write(f"td3bc_alpha           = {self.td3bc_alpha}\n")
            f.write(f"lr_actor              = {self.lr_actor}\n")
            f.write(f"lr_critic             = {self.lr_critic}\n")
            f.write(f"batch_size            = {self.batch_size}\n")
            f.write(f"actor_hidden_dim      = {self.policy_hidden_dim}\n")
            f.write(f"critic_hidden_dim     = {self.critic_hidden_dim}\n")
            f.write(f"input_dim             = {self.policy_input_dim}\n")
            f.write(f"reward_cte_weight     = {self.reward_cte_weight}\n")
            f.write(f"reward_cte_norm       = {self.reward_cte_norm}\n")
            f.write(f"reward_improve_weight = {self.reward_improve_weight}\n")
            f.write(f"reward_speed_weight   = {self.reward_speed_weight}\n")
            f.write(f"reward_action_weight  = {self.reward_action_weight}\n")
            f.write(f"train_device          = {self.train_device}\n")

        self.get_logger().info(f"[train] Logs written to: {csv_path} | {txt_path}")


# ──────────────────────────────────────────────────────────────────────────────
# Entrypoint
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrainRLResidual()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rclpy.ok():
            rclpy.logging.get_logger("train_rl_residual").error(f"Error fatal: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()