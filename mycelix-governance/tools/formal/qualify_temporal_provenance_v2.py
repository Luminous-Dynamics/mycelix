#!/usr/bin/env python3
"""Exact-head temporal provenance qualification v2.

This runner reuses deterministic execution helpers from qualify_temporal_provenance.py
but owns the final evidence decision. It corrects the closure/fault negative control:
the mutant keeps fault=TRUE while suppressing the contradiction record, so
FaultHasClosureContradiction must actually fire.
"""
from __future__ import annotations

import argparse
import datetime as dt
import importlib.util
import json
import os
import platform
import sys
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
TOOLS = SCRIPT.parent
BASE_PATH = TOOLS / "qualify_temporal_provenance.py"
PINS = TOOLS / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-temporal-provenance-qualification.yml"
SPEC_ROOT = REPO / "mycelix-governance" / "specs"
CRATE_ROOT = REPO / "mycelix-governance" / "crates" / "constitutional-temporal-provenance"

INDEPENDENT_CFG = SPEC_ROOT / "ConstitutionalEvidenceClosure.independent.cfg"
INDEPENDENT_REACH_CFG = SPEC_ROOT / "ConstitutionalEvidenceClosure.reach-independent-incomparable.cfg"
STRICT_TEST = CRATE_ROOT / "tests" / "strict_observation.rs"
INDEPENDENT_TEST = CRATE_ROOT / "tests" / "independent_order.rs"
ABSTRACTION_NOTE = REPO / "mycelix-governance" / "docs" / "CONSTITUTIONAL_TEMPORAL_PROVENANCE_ABSTRACTION_NOTES.md"


def load_base():
    spec = importlib.util.spec_from_file_location("mycelix_temporal_base", BASE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load base temporal qualifier")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def unique_paths(paths: list[Path]) -> list[Path]:
    seen: set[str] = set()
    out: list[Path] = []
    for path in paths:
        key = str(path.resolve())
        if key not in seen:
            seen.add(key)
            out.append(path)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tla-jar", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--expected-head", required=True)
    ap.add_argument("--java", default="java")
    ap.add_argument("--cargo", default="cargo")
    args = ap.parse_args()

    base = load_base()
    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    inputs = unique_paths([
        *base.INPUTS,
        STRICT_TEST,
        INDEPENDENT_TEST,
        ABSTRACTION_NOTE,
        INDEPENDENT_CFG,
        INDEPENDENT_REACH_CFG,
        BASE_PATH,
        PINS,
        SCRIPT,
        WORKFLOW,
    ])

    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-temporal-provenance-qualification.v2",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        pre = base.snapshot(inputs)
        actual_tla = base.sha256_file(args.tla_jar)
        if actual_tla != pins["tla2tools"]["sha256"]:
            raise RuntimeError("TLA+ JAR SHA-256 mismatch")

        git_rc, git_head = base.run(["git", "rev-parse", "HEAD"], cwd=REPO)
        if git_rc != 0 or git_head.strip() != args.expected_head:
            raise RuntimeError(
                f"raw-head mismatch: expected={args.expected_head} actual={git_head.strip()}"
            )

        java_rc, java_version = base.run([args.java, "-version"])
        cargo_rc, cargo_version = base.run([args.cargo, "--version"])
        rustc_rc, rustc_version = base.run(["rustc", "--version"])
        if java_rc != 0 or cargo_rc != 0 or rustc_rc != 0:
            raise RuntimeError("tool version probe failed")

        receipt.update({
            "git_head": git_head.strip(),
            "environment": {
                "python": sys.version,
                "platform": platform.platform(),
                "java": java_version.strip(),
                "cargo": cargo_version.strip(),
                "rustc": rustc_version.strip(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
            },
            "tla2tools": {
                "version": pins["tla2tools"]["version"],
                "sha256": actual_tla,
            },
            "preflight_input_sha256": pre,
        })

        rust = base.run_rust_tests(args.cargo, args.expected_head, outdir)
        spec_text = base.TLA_SPEC.read_text(encoding="utf-8")

        safety = [
            base.run_tlc_case(
                label=f"safety-{name}",
                spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=None,
            )
            for name, path in base.SAFETY_CONFIGS.items()
        ]

        reachability = [
            base.run_tlc_case(
                label=f"reach-{name}",
                spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=expected,
            )
            for name, (path, expected) in base.REACHABILITY_CONFIGS.items()
        ]

        detection_cfg = base.SAFETY_CONFIGS["detection"].read_text(encoding="utf-8")
        witnessed_cfg = base.SAFETY_CONFIGS["witnessed"].read_text(encoding="utf-8")

        fail_open_detection = spec_text.replace(
            'ClosureAllowed == Profile # "DetectionOnly"',
            'ClosureAllowed == TRUE',
            1,
        )
        if fail_open_detection == spec_text:
            raise RuntimeError("negative control mutation failed: ClosureAllowed")

        contradiction_marker = r"/\ closureContradictions' = closureContradictions \cup {f}"
        contradiction_replacement = r"/\ closureContradictions' = closureContradictions"
        missing_contradiction_record = spec_text.replace(
            contradiction_marker,
            contradiction_replacement,
            1,
        )
        if missing_contradiction_record == spec_text:
            raise RuntimeError("negative control mutation failed: closureContradictions record")

        negative_controls = [
            base.run_tlc_case(
                label="negative-detection-closure",
                spec_text=fail_open_detection,
                cfg_text=detection_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="DetectionOnlyHasNoClosure",
            ),
            base.run_tlc_case(
                label="negative-fault-without-contradiction-record",
                spec_text=missing_contradiction_record,
                cfg_text=witnessed_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="FaultHasClosureContradiction",
            ),
        ]

        independent_safety = base.run_tlc_case(
            label="safety-independent",
            spec_text=spec_text,
            cfg_text=INDEPENDENT_CFG.read_text(encoding="utf-8"),
            java=args.java,
            jar=args.tla_jar.resolve(),
            outdir=outdir,
            expected_violation=None,
        )
        independent_reachability = base.run_tlc_case(
            label="reach-independent-incomparable",
            spec_text=spec_text,
            cfg_text=INDEPENDENT_REACH_CFG.read_text(encoding="utf-8"),
            java=args.java,
            jar=args.tla_jar.resolve(),
            outdir=outdir,
            expected_violation="NeverIndependentIncomparableAccepted",
        )

        post = base.snapshot(inputs)
        immutable = pre == post
        receipt.update({
            "rust": rust,
            "tlc_safety": safety,
            "tlc_reachability": reachability,
            "tlc_negative_controls": negative_controls,
            "independent_safety": independent_safety,
            "independent_reachability": independent_reachability,
            "strict_observation_test_included": STRICT_TEST.is_file(),
            "independent_order_test_included": INDEPENDENT_TEST.is_file(),
            "abstraction_note_included": ABSTRACTION_NOTE.is_file(),
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            rust["passed"]
            and all(x["passed"] for x in safety)
            and all(x["passed"] for x in reachability)
            and all(x["passed"] for x in negative_controls)
            and independent_safety["passed"]
            and independent_reachability["passed"]
            and immutable
        )
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        try:
            post = base.snapshot(inputs)
            receipt["postflight_input_sha256"] = post
            receipt["postflight_immutable"] = (
                receipt.get("preflight_input_sha256") == post
            )
        except Exception as post_exc:
            receipt["postflight_error"] = f"{type(post_exc).__name__}: {post_exc}"

    receipt["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    receipt_path = outdir / "qualification-receipt-v2.json"
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    manifest = base.artifact_manifest(outdir)
    (outdir / "artifact-manifest-v2.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"passed": receipt["passed"], "receipt": str(receipt_path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
