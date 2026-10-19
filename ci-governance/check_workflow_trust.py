#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

PINNED_REMOTE_ACTION = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_./-]+)?@[0-9a-fA-F]{40}$")
JOB_KEY = re.compile(r"^([A-Za-z0-9_.-]+):\s*(?:#.*)?$")
MAPPING_ITEM = re.compile(r"^([A-Za-z0-9_.-]+):\s*([^#]*?)\s*(?:#.*)?$")


class PolicyError(Exception):
    pass


def indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def meaningful(text: str):
    out = []
    for n, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw:
            raise PolicyError(f"line {n}: tabs are forbidden")
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        out.append((n, raw))
    return out


def parse_mapping(items, start, parent_indent):
    out = {}
    i = start
    while i < len(items):
        n, line = items[i]
        if indent(line) <= parent_indent:
            break
        m = MAPPING_ITEM.match(line.strip())
        if not m:
            raise PolicyError(f"line {n}: unsupported mapping syntax")
        key, value = m.groups()
        value = value.strip().strip("'\"")
        if not value:
            raise PolicyError(f"line {n}: nested permission values are unsupported")
        out[key] = value
        i += 1
    return out, i


def parse_workflow(text: str):
    if re.search(r"(^|\s)[&*][A-Za-z0-9_-]+", text):
        raise PolicyError("YAML anchors/aliases are forbidden in trust-policy surface")
    items = meaningful(text)
    top_permissions = None
    permissions_seen = 0
    jobs_index = None

    for idx, (n, line) in enumerate(items):
        if indent(line) != 0:
            continue
        t = line.strip()
        if t.startswith("permissions:"):
            permissions_seen += 1
            if permissions_seen > 1:
                raise PolicyError(f"line {n}: duplicate workflow permissions are forbidden")
            if t != "permissions:":
                raise PolicyError(f"line {n}: inline workflow permissions are forbidden")
            top_permissions, _ = parse_mapping(items, idx + 1, 0)
        elif t == "jobs:":
            jobs_index = idx

    if jobs_index is None:
        raise PolicyError("workflow has no top-level jobs block")

    jobs = {}
    i = jobs_index + 1
    while i < len(items):
        n, line = items[i]
        ind = indent(line)
        if ind == 0:
            break
        if ind != 2:
            i += 1
            continue
        m = JOB_KEY.match(line.strip())
        if not m:
            raise PolicyError(f"line {n}: unsupported job key syntax")
        job_name = m.group(1)
        job_permissions = None
        job_permissions_seen = 0
        steps = []
        i += 1
        while i < len(items):
            n2, line2 = items[i]
            ind2 = indent(line2)
            if ind2 <= 2:
                break
            t2 = line2.strip()
            if ind2 == 4 and t2.startswith("permissions:"):
                job_permissions_seen += 1
                if job_permissions_seen > 1:
                    raise PolicyError(f"line {n2}: duplicate job permissions are forbidden")
                if t2 != "permissions:":
                    raise PolicyError(f"line {n2}: inline job permissions are forbidden")
                job_permissions, i = parse_mapping(items, i + 1, 4)
                continue
            if ind2 == 4 and t2 == "steps:":
                i += 1
                current = None
                while i < len(items):
                    ns, ls = items[i]
                    inds = indent(ls)
                    if inds <= 4:
                        break
                    ts = ls.strip()
                    if inds == 6 and ts.startswith("- "):
                        current = {"uses": None, "with": {}}
                        steps.append(current)
                        remainder = ts[2:].strip()
                        if remainder.startswith("uses:"):
                            current["uses"] = remainder.split(":", 1)[1].strip().strip("'\"")
                        i += 1
                        continue
                    if current is None:
                        raise PolicyError(f"line {ns}: step content before step item")
                    if inds == 8 and ts.startswith("uses:"):
                        if current["uses"] is not None:
                            raise PolicyError(f"line {ns}: duplicate uses key is forbidden")
                        current["uses"] = ts.split(":", 1)[1].strip().strip("'\"")
                        i += 1
                        continue
                    if inds == 8 and ts == "with:":
                        i += 1
                        while i < len(items):
                            nw, lw = items[i]
                            indw = indent(lw)
                            if indw <= 8:
                                break
                            if indw != 10:
                                raise PolicyError(f"line {nw}: unsupported nested with syntax")
                            mw = MAPPING_ITEM.match(lw.strip())
                            if not mw:
                                raise PolicyError(f"line {nw}: unsupported with mapping syntax")
                            k, v = mw.groups()
                            if k in current["with"]:
                                raise PolicyError(f"line {nw}: duplicate with key {k!r} is forbidden")
                            current["with"][k] = v.strip().strip("'\"")
                            i += 1
                        continue
                    i += 1
                continue
            i += 1
        jobs[job_name] = {"permissions": job_permissions, "steps": steps}
    return {"permissions": top_permissions, "jobs": jobs}


def check(workflow_path: Path, policy_path: Path):
    policy = json.loads(policy_path.read_text())
    if policy.get("schema") != "mycelix.ci.trust-policy.v0.1":
        raise PolicyError("unsupported policy schema")
    wf = parse_workflow(workflow_path.read_text())
    findings = []
    expected_default = policy["workflow_default_permissions"]
    if wf["permissions"] != expected_default:
        findings.append(f"workflow permissions must equal {expected_default!r}; observed {wf['permissions']!r}")

    for job_name, job in wf["jobs"].items():
        effective = job["permissions"] if job["permissions"] is not None else wf["permissions"]
        if effective is None:
            findings.append(f"job {job_name}: permissions are implicit")
            continue
        for permission, level in effective.items():
            if str(level).lower() == "write":
                findings.append(f"job {job_name}: forbidden write permission {permission}: write")

    for job_name, required in policy.get("job_required_permissions", {}).items():
        job = wf["jobs"].get(job_name)
        if job is None:
            findings.append(f"required job missing: {job_name}")
            continue
        effective = job["permissions"] if job["permissions"] is not None else wf["permissions"]
        if effective != required:
            findings.append(
                f"job {job_name}: permissions must equal {required!r}; observed {effective!r}"
            )

    for job_name in sorted(set(policy.get("critical_jobs", []))):
        job = wf["jobs"].get(job_name)
        if job is None:
            findings.append(f"critical job missing: {job_name}")
            continue
        for idx, step in enumerate(job["steps"], 1):
            uses = step["uses"]
            if not uses or uses.startswith("./"):
                continue
            if policy.get("require_full_sha_for_remote_actions", True) and not PINNED_REMOTE_ACTION.fullmatch(uses):
                findings.append(f"job {job_name} step {idx}: remote action is not pinned to a 40-hex commit: {uses}")
            if uses.startswith("actions/checkout@") and policy.get("require_checkout_persist_credentials_false", True):
                if step["with"].get("persist-credentials") != "false":
                    findings.append(f"job {job_name} step {idx}: checkout must set persist-credentials: false")

    return {
        "schema": "mycelix.ci.trust-policy.report.v0.1",
        "workflow": str(workflow_path),
        "policy": str(policy_path),
        "status": "PASS" if not findings else "FAIL",
        "findings": findings,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("workflow", type=Path)
    ap.add_argument("--policy", type=Path, default=Path(__file__).with_name("trust-policy-v0.1.json"))
    args = ap.parse_args()
    try:
        report = check(args.workflow, args.policy)
    except Exception as exc:
        report = {
            "schema": "mycelix.ci.trust-policy.report.v0.1",
            "workflow": str(args.workflow),
            "policy": str(args.policy),
            "status": "ERROR",
            "findings": [str(exc)],
        }
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
