#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNNER_FILES = (
    "qcap4.py",
    "qcap_canon.py",
    "qcap_manifest.py",
    "qcap3_limits.py",
    "qcap4_containment.py",
    "qcap4_context.py",
    "qcap4_receipt.py",
    "qcap4_repo.py",
    "qcap4_launch_wrapper.py",
    "qcap4_exec.py",
)


def _bootstrap_fail(message):
    print("qcap4_bootstrap_error=" + message, file=sys.stderr)
    raise SystemExit(20)


def _reference_bootstrap():
    if (
        not sys.flags.isolated
        or not sys.dont_write_bytecode
        or not getattr(sys.flags, "safe_path", False)
    ):
        _bootstrap_fail("reference adapter requires python -I -B")
    for name in RUNNER_FILES:
        path = HERE / name
        if path.is_symlink() or not path.is_file():
            _bootstrap_fail("runner source closure invalid: " + name)
    for path in HERE.rglob("*.pyc"):
        _bootstrap_fail("bytecode cache forbidden: " + str(path.relative_to(HERE)))
    stdlib = getattr(sys, "stdlib_module_names", set())
    for path in HERE.iterdir():
        stem = path.stem if path.is_file() else path.name
        if stem in stdlib and (
            (path.is_file() and path.suffix == ".py") or path.is_dir()
        ):
            _bootstrap_fail("stdlib shadow forbidden: " + path.name)


if __name__ == "__main__":
    _reference_bootstrap()
sys.path.insert(0, str(HERE))

from qcap_canon import CapsuleError
from qcap_manifest import validate_manifest
from qcap3_limits import limits_digest, limits_ref, validate_limits
from qcap4_containment import containment_digest, containment_ref, validate_containment_profile
from qcap4_receipt import verify_receipt_v4
import qcap4_exec


def runner_commitment():
    digest = hashlib.sha256()
    digest.update(b"MYCELIX_QCAP_RUNNER_V4\0")
    for name in RUNNER_FILES:
        content = (HERE / name).read_bytes()
        encoded_name = name.encode()
        digest.update(len(encoded_name).to_bytes(4, "big"))
        digest.update(encoded_name)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def run_capsule(
    manifest,
    root,
    repo,
    repository_identity,
    attempt_id,
    context,
    limits,
    containment_profile,
    containment_root,
):
    before = runner_commitment()
    receipt = qcap4_exec.run_capsule_v4(
        manifest,
        root,
        repo,
        repository_identity,
        attempt_id,
        context,
        limits,
        containment_profile,
        containment_root,
        before,
    )
    if runner_commitment() != before:
        raise CapsuleError("runner self-integrity changed during attempt")
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="cmd", required=True)

    limits_parser = subparsers.add_parser("limits")
    limits_parser.add_argument("limits")

    containment_parser = subparsers.add_parser("containment-profile")
    containment_parser.add_argument("containment_profile")

    verify_parser = subparsers.add_parser("verify-receipt")
    verify_parser.add_argument("manifest")
    verify_parser.add_argument("limits")
    verify_parser.add_argument("containment_profile")
    verify_parser.add_argument("receipt")

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("manifest")
    run_parser.add_argument("--root", required=True)
    run_parser.add_argument("--subject-repo", required=True)
    run_parser.add_argument("--repository-identity", required=True)
    run_parser.add_argument("--attempt-id", required=True)
    run_parser.add_argument("--execution-context", required=True)
    run_parser.add_argument("--execution-limits", required=True)
    run_parser.add_argument("--containment-profile", required=True)
    run_parser.add_argument("--containment-root", required=True)
    run_parser.add_argument("--receipt-out")
    args = parser.parse_args(argv)

    try:
        if args.cmd == "limits":
            limits = load(args.limits)
            validate_limits(limits)
            print("execution_limits_digest=" + limits_digest(limits))
            print("execution_limits_ref=" + json.dumps(limits_ref(limits), sort_keys=True))
            return 0
        if args.cmd == "containment-profile":
            profile = load(args.containment_profile)
            validate_containment_profile(profile)
            print("containment_profile_digest=" + containment_digest(profile))
            print("containment_profile_ref=" + json.dumps(containment_ref(profile), sort_keys=True))
            return 0

        manifest = load(args.manifest)
        limits = load(args.limits if args.cmd == "verify-receipt" else args.execution_limits)
        profile = load(args.containment_profile)
        if args.cmd == "verify-receipt":
            validate_manifest(manifest)
            verify_receipt_v4(load(args.receipt), manifest, limits, profile)
            print("receipt_integrity=PASS")
            return 0

        receipt = run_capsule(
            manifest,
            args.root,
            args.subject_repo,
            args.repository_identity,
            args.attempt_id,
            load(args.execution_context),
            limits,
            profile,
            args.containment_root,
        )
        encoded = json.dumps(receipt, sort_keys=True, indent=2) + "\n"
        if args.receipt_out:
            Path(args.receipt_out).write_text(encoded)
        else:
            sys.stdout.write(encoded)
        if receipt["verdict"] == "CompletedConjunctivePass":
            return 0
        if receipt["verdict"] == "CompletedConjunctiveFail":
            return 10
        return 20
    except (CapsuleError, ValueError, json.JSONDecodeError) as error:
        print("qcap4_error=" + str(error), file=sys.stderr)
        return 20


if __name__ == "__main__":
    raise SystemExit(main())
