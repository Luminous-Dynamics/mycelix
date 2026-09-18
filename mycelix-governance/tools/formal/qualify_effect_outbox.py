#!/usr/bin/env python3
"""Exact-head qualification for MYC-CONST-003D1A."""
from __future__ import annotations

import argparse, datetime as dt, hashlib, json, os, platform, re, subprocess, tempfile
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
GOV = REPO / "mycelix-governance"
SPEC = GOV / "specs"
CORE = SPEC / "ConstitutionalEffectOutbox.tla"
SAFETY = SPEC / "ConstitutionalEffectOutboxSafety.tla"
CFG = SPEC / "ConstitutionalEffectOutboxSafety.cfg"
DOC = GOV / "docs" / "CONSTITUTIONAL_EFFECT_OUTBOX_V0_1.md"
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-effect-outbox-qualification.yml"
INPUTS = [CORE, SAFETY, CFG, DOC, PINS, SCRIPT, WORKFLOW]
BOUNDS = [6, 8, 10]
REACH = [
    "CrashAfterPrepareReached", "CrashAfterCommitBeforeEffectReached",
    "UnknownOutcomeWithEffectReached", "UnknownOutcomeNoEffectReached",
    "ReconcileSuccessReached", "ReconcileNoEffectReached", "DuplicateDeliveryReached",
    "ReceiptAckLostReached", "ContradictionHaltReached",
]
MUTANTS = {
    "effect-before-commit": "EffectRequiresConstitutionalCommit",
    "unstable-retry-identity": "RetryDoesNotChangeOperationIdentity",
    "retry-during-unknown": "UnknownOutcomeBlocksRetry",
    "receipt-before-observation": "ReceiptRequiresObservedEffect",
    "crash-erases-commit": "CrashPreservesDurableHistory",
    "non-idempotent-delivery": "LogicalEffectAtMostOnce",
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path: Path) -> str:
    try: return str(path.resolve().relative_to(REPO.resolve()))
    except ValueError: return str(path.resolve())


def snapshot() -> dict[str, str]:
    out = {}
    for path in INPUTS:
        if not path.is_file(): raise RuntimeError(f"missing input: {path}")
        out[rel(path)] = sha(path)
    return out


def run(cmd: list[str], cwd: Path | None = None) -> tuple[int, str]:
    p = subprocess.run(cmd, cwd=str(cwd) if cwd else None, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                       errors="replace", check=False)
    return p.returncode, p.stdout


def stats(text: str) -> dict[str, int | None]:
    def g(pattern: str):
        m = re.search(pattern, text)
        return int(m.group(1).replace(",", "")) if m else None
    return {
        "states_generated": g(r"([0-9,]+) states generated"),
        "distinct_states": g(r"([0-9,]+) distinct states found"),
        "depth": g(r"depth of the complete state graph search is ([0-9,]+)"),
    }


def violated(text: str) -> list[str]:
    return sorted(set(re.findall(r"Invariant ([A-Za-z_][A-Za-z0-9_]*) is violated", text)))


def cfg(bound: int, invariants: list[str]) -> str:
    return "SPECIFICATION Spec\n\nCONSTANT\n    MaxStep = %d\n\nINVARIANTS\n%s\n" % (
        bound, "\n".join(f"    {x}" for x in invariants)
    )


def tlc(label: str, core: str, safety: str, config: str, java: str, jar: Path,
        outdir: Path, expected: str | None) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"outbox-{label}-") as td:
        w = Path(td)
        (w / CORE.name).write_text(core, encoding="utf-8")
        (w / SAFETY.name).write_text(safety, encoding="utf-8")
        c = w / "ConstitutionalEffectOutboxSafety.cfg"
        c.write_text(config, encoding="utf-8")
        cmd = [java, "-cp", str(jar), "tlc2.TLC", "-workers", "1", "-deadlock",
               "-config", c.name, SAFETY.name]
        rc, output = run(cmd, w)
    log = outdir / "tlc" / f"{label}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(output, encoding="utf-8")
    s, v = stats(output), violated(output)
    explored = (s["states_generated"] or 0) > 0 and (s["distinct_states"] or 0) > 0
    passed = (rc == 0 and explored and not v and "No error has been found" in output) if expected is None else (explored and v == [expected])
    return {"label": label, "returncode": rc, "stats": s, "violations": v,
            "expected": "SAFETY_PASS" if expected is None else f"EXACT_VIOLATION:{expected}",
            "log": rel(log), "log_sha256": sha(log), "passed": passed}


def segment(text: str, start: str, end: str, replacements: list[tuple[str, str]]) -> str:
    a, b = text.find(start), text.find(end, text.find(start) + len(start))
    if a < 0 or b < 0: raise RuntimeError(f"missing mutation segment {start}")
    part, old = text[a:b], text[a:b]
    for x, y in replacements: part = part.replace(x, y, 1)
    if part == old: raise RuntimeError(f"mutation failed {start}")
    return text[:a] + part + text[b:]


def mutate(core: str, label: str) -> str:
    if label == "unstable-retry-identity":
        return segment(core, "StartRequest(op) ==", "ExternalDeliver(op) ==",
                       [("/\\ inFlight' = op", "/\\ inFlight' = IF op = \"A\" THEN \"B\" ELSE \"A\"")])
    if label == "retry-during-unknown":
        return segment(core, "LoseOrTimeoutAck(op) ==", "ReconcileSuccess(op) ==",
                       [("/\\ worker' = NoOp", "/\\ worker' = op"),
                        ("/\\ inFlight' = NoOp", "/\\ inFlight' = op")])
    if label == "receipt-before-observation":
        return segment(core, "CommitReceipt(op) ==", "AcknowledgeCaller(op) ==",
                       [("/\\ phase[op] = \"EffectObserved\"", "/\\ phase[op] = \"EffectPending\""),
                        ("/\\ observedEffect = op", "/\\ TRUE")])
    if label == "crash-erases-commit":
        return segment(core, "Crash ==", "ObserveContradiction(op) ==",
                       [("/\\ crashCount' = crashCount + 1", "/\\ committedOp' = NoOp\n    /\\ crashCount' = crashCount + 1"),
                        ("UNCHANGED <<phase, committedOp, outbox", "UNCHANGED <<phase, outbox")])
    if label == "non-idempotent-delivery":
        return segment(core, "ExternalDeliver(op) ==", "ReceiveSuccessAck(op) ==",
                       [("/\\ effectCount' = 1", "/\\ effectCount' = effectCount + 1")])
    if label == "effect-before-commit":
        action = r'''
MutantEffectWithoutCommit(op) ==
    /\ Advanceable
    /\ committedOp = NoOp
    /\ phase[op] = "Absent"
    /\ externalEffect' = op
    /\ effectCount' = 1
    /\ effectHistory' = effectHistory \cup {op}
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, observedEffect, receipt,
                   integrityHalt, worker, inFlight, callerAck,
                   committedHistory, receiptHistory, callerAckHistory,
                   deliveryAttempts, crashCount, reconciledSuccess,
                   reconciledNoEffect, contradictionSeen>>

'''
        if "Next ==\n" not in core: raise RuntimeError("missing Next")
        core = core.replace("Next ==\n", action + "Next ==\n", 1)
        anchor = "Next ==\n       (\\E op \\in Ops : Prepare(op))"
        repl = "Next ==\n       (\\E op \\in Ops : MutantEffectWithoutCommit(op))\n    \\/ (\\E op \\in Ops : Prepare(op))"
        if anchor not in core: raise RuntimeError("missing Next disjunction")
        return core.replace(anchor, repl, 1)
    raise RuntimeError(label)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tla-jar", type=Path, required=True)
    for name in ("head", "semantic-head", "lifecycle-head", "closure-head", "temporal-head"):
        ap.add_argument(f"--expected-{name}", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--java", default="java")
    a = ap.parse_args()
    out = a.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {"schema": "mycelix.constitutional-effect-outbox-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "passed": False}
    try:
        pins = json.loads(PINS.read_text(encoding="utf-8")); pre = snapshot()
        if sha(a.tla_jar) != pins["tla2tools"]["sha256"]: raise RuntimeError("TLA+ JAR SHA-256 mismatch")
        expected = [a.expected_head, a.expected_semantic_head, a.expected_lifecycle_head,
                    a.expected_closure_head, a.expected_temporal_head]
        refs = ["HEAD", "HEAD^", "HEAD^^", "HEAD^^^", "HEAD^^^^"]
        lineage = {}
        for ref, want in zip(refs, expected):
            rc, text = run(["git", "rev-parse", ref], REPO); got = text.strip()
            if rc or got != want: raise RuntimeError(f"lineage mismatch {ref}: {got} != {want}")
            lineage[ref] = got
        rc, java_version = run([a.java, "-version"])
        if rc: raise RuntimeError("java version probe failed")
        core, safety = CORE.read_text(encoding="utf-8"), SAFETY.read_text(encoding="utf-8")
        invariants = ["TypeOK", "AtMostOneCommittedOpPerUse", "EffectRequiresConstitutionalCommit",
            "ReceiptRequiresObservedEffect", "ExternalEffectIdentityMatchesCommittedOp",
            "RetryDoesNotChangeOperationIdentity", "UnknownOutcomeBlocksConflictingOp",
            "UnknownOutcomeBlocksRetry", "CrashPreservesDurableHistory", "ReceiptMonotonic",
            "IntegrityHaltDoesNotEraseEffect", "CallerAckIsNotConstitutionalState",
            "LogicalEffectAtMostOnce", "OutboxBeforeEffect"]
        matrix = [tlc(f"safety-{n}", core, safety, cfg(n, invariants), a.java, a.tla_jar.resolve(), out, None) for n in BOUNDS]
        reach = [tlc(f"reach-{p}", core, safety, cfg(10, ["Never" + p]), a.java,
                     a.tla_jar.resolve(), out, "Never" + p) for p in REACH]
        mutants = [tlc(f"mutant-{m}", mutate(core, m), safety, cfg(10, [target]), a.java,
                       a.tla_jar.resolve(), out, target) for m, target in MUTANTS.items()]
        d = [x["stats"]["distinct_states"] for x in matrix]
        sensitivity = {"bounds": BOUNDS, "distinct_states": d,
                       "nondecreasing": all(isinstance(x, int) and isinstance(y, int) and x <= y for x, y in zip(d, d[1:]))}
        doc = DOC.read_text(encoding="utf-8")
        terms = ["UnknownOutcome", "logical idempotency assumption", "physical exactly-once", "CommitAndEnqueue", "EffectPending"]
        contract = {"missing_terms": [t for t in terms if t not in doc]}; contract["passed"] = not contract["missing_terms"]
        post = snapshot(); immutable = pre == post
        receipt.update({"git_lineage": lineage,
            "environment": {"java": java_version.strip(), "platform": platform.platform(), "github_run_id": os.environ.get("GITHUB_RUN_ID")},
            "tla2tools": {"version": pins["tla2tools"]["version"], "sha256": sha(a.tla_jar), "workers": 1},
            "preflight_input_sha256": pre, "safety_matrix": matrix, "bound_sensitivity": sensitivity,
            "reachability": reach, "negative_controls": mutants, "semantic_contract": contract,
            "postflight_input_sha256": post, "postflight_immutable": immutable})
        receipt["passed"] = all(x["passed"] for x in matrix + reach + mutants) and sensitivity["nondecreasing"] and contract["passed"] and immutable
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
    receipt["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    path = out / "qualification-receipt.json"; path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {str(p.relative_to(out)): {"sha256": sha(p), "size": p.stat().st_size}
                for p in sorted(x for x in out.rglob("*") if x.is_file() and x.name != "artifact-manifest.json")}
    (out / "artifact-manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": receipt["passed"], "receipt": str(path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1

if __name__ == "__main__": raise SystemExit(main())
