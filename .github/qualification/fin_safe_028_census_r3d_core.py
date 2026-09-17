#!/usr/bin/env python3
import hashlib
import json
import pathlib
import re
import subprocess
from collections import Counter, defaultdict

SOURCE_SHA = "f7b43e3f9718e2dc2445376916bd4d5010fbc4ea"
TOKENS = (
    "require_consciousness",
    "gate_consciousness",
    "GovernanceRequirement",
    "require_civic",
    "gate_civic",
    "CivicRequirement",
)
GATE_TOKENS = {
    "require_consciousness",
    "gate_consciousness",
    "require_civic",
    "gate_civic",
}
EXPECTED_TOKEN_COUNTS = {
    "CivicRequirement": 71,
    "GovernanceRequirement": 93,
    "gate_civic": 152,
    "gate_consciousness": 42,
    "require_civic": 867,
    "require_consciousness": 57,
}
EXPECTED_CLASS_COUNTS = {
    "definition": 18,
    "documentation": 17,
    "executable_or_source_reference": 1096,
    "import": 13,
    "other_source": 22,
    "source_comment": 100,
    "test": 16,
}
EXPECTED_TOTAL_MATCHES = 1282
EXPECTED_EXECUTABLE_GATE_REFERENCES = 979
EXPECTED_CALL_LIKE_SITES = 946
EXPECTED_RECORD_SET_SHA256 = "d71ccbdde6dbb8de6792189096015252948b552834dfd8b1938a85ce21e85d3a"
EXPECTED_GATE_SET_SHA256 = "ed166dcfd0c70fde8615506454aed715ca324472b9e29f4092605553d2d61ad0"
EXPECTED_CALL_SET_SHA256 = "3a3d05b44313be7794dd2cb329a697e237b7149c51163440994261a1a0be3f31"
REGULAR_MODES = {"100644", "100755"}
TOPOLOGY_MODES = {"120000": "symlink", "160000": "gitlink"}
EXPECTED_HEALTH_GITLINK = "6b238396e80e7e5709c826e32f4eb48d0dae83f8"
R2_FIELDS = ("classification", "line", "path", "text", "token")


def run(*args: str) -> bytes:
    return subprocess.check_output(args)


def canonical_sha256(value) -> str:
    raw = (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()
    return hashlib.sha256(raw).hexdigest()


def classify(path: pathlib.PurePosixPath, line: str, token: str) -> str:
    # Byte-for-byte semantic copy of the qualified R2 classifier. R3d changes
    # only the content-identity projection and explicit repository topology.
    p = path.as_posix()
    lower = p.lower()
    stripped = line.strip()
    if p.startswith(".github/"):
        return "workflow"
    if "/docs/" in f"/{lower}" or lower.endswith((".md", ".txt", ".rst")):
        return "documentation"
    if (
        "/tests/" in f"/{lower}"
        or lower.startswith("tests/")
        or lower.endswith("_test.rs")
        or "/test_" in lower
    ):
        return "test"
    if path.suffix == ".rs":
        def_pat = re.compile(
            r"(?:^|\s)(?:pub(?:\([^)]*\))?\s+)?(?:async\s+)?(?:fn|struct|enum|type|trait|const)\s+"
            + re.escape(token)
            + r"\b"
        )
        if def_pat.search(stripped):
            return "definition"
        if re.match(r"^(?:pub\s+)?use\b", stripped) or (
            token in stripped and stripped.startswith("use ")
        ):
            return "import"
        if stripped.startswith(("//", "///", "//!", "/*", "*")):
            return "source_comment"
        return "executable_or_source_reference"
    return "other_source"


def call_like(line: str, token: str, classification: str) -> bool:
    return (
        classification == "executable_or_source_reference"
        and token in GATE_TOKENS
        and re.search(r"\b" + re.escape(token) + r"\s*\(", line) is not None
    )


def read_index():
    index = {}
    raw = run("git", "ls-files", "-s", "-z")
    for item in raw.split(b"\0"):
        if not item:
            continue
        meta, raw_path = item.split(b"\t", 1)
        mode, object_sha, stage = meta.decode("ascii").split()
        if stage != "0":
            raise SystemExit(f"non-stage-0 index entry for {raw_path!r}: stage={stage}")
        path = raw_path.decode("utf-8", errors="surrogateescape")
        index[path] = {"mode": mode, "object_sha": object_sha}
    return index


def verified_regular_text(path_text: str, expected_blob_sha: str):
    data = pathlib.Path(path_text).read_bytes()
    actual_blob_sha = subprocess.check_output(
        ["git", "hash-object", "--stdin"], input=data
    ).decode("ascii").strip()
    if actual_blob_sha != expected_blob_sha:
        raise SystemExit(
            f"worktree/index blob mismatch for {path_text}: "
            f"{actual_blob_sha} != {expected_blob_sha}"
        )
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


def project_r2(records):
    return [{key: record[key] for key in R2_FIELDS} for record in records]


def dedupe(items):
    grouped = defaultdict(list)
    representative = {}
    for record in items:
        key = (record["blob_sha"], record["line"], record["token"])
        grouped[key].append(record["path"])
        representative.setdefault(key, record)

    output = []
    for key in sorted(grouped):
        record = representative[key]
        aliases = sorted(set(grouped[key]))
        output.append(
            {
                "blob_sha": record["blob_sha"],
                "line": record["line"],
                "token": record["token"],
                "text": record["text"],
                "classification": record["classification"],
                "call_like": record["call_like"],
                "paths": aliases,
                "path_alias_count": len(aliases),
            }
        )
    return output


def main() -> None:
    actual_head = run("git", "rev-parse", "HEAD").decode().strip()
    if actual_head != SOURCE_SHA:
        raise SystemExit(f"wrong source checkout: {actual_head} != {SOURCE_SHA}")

    index = read_index()
    health = index.get("mycelix-health")
    if health != {"mode": "160000", "object_sha": EXPECTED_HEALTH_GITLINK}:
        raise SystemExit(f"unexpected mycelix-health topology: {health!r}")

    records = []
    topology = []
    regular_files_verified = 0
    utf8_files_scanned = 0

    for path_text, meta in sorted(index.items()):
        mode = meta["mode"]
        object_sha = meta["object_sha"]
        if mode in TOPOLOGY_MODES:
            topology.append(
                {
                    "path": path_text,
                    "mode": mode,
                    "kind": TOPOLOGY_MODES[mode],
                    "object_sha": object_sha,
                }
            )
            continue
        if mode not in REGULAR_MODES:
            raise SystemExit(f"unsupported tracked mode {mode} for {path_text}")

        text = verified_regular_text(path_text, object_sha)
        regular_files_verified += 1
        if text is None:
            continue
        utf8_files_scanned += 1
        path = pathlib.PurePosixPath(path_text)
        for lineno, line in enumerate(text.splitlines(), start=1):
            for token in TOKENS:
                if token not in line:
                    continue
                classification = classify(path, line, token)
                records.append(
                    {
                        "token": token,
                        "path": path_text,
                        "mode": mode,
                        "blob_sha": object_sha,
                        "line": lineno,
                        "text": line.rstrip(),
                        "classification": classification,
                        "call_like": call_like(line, token, classification),
                    }
                )

    records.sort(key=lambda r: (r["path"], r["line"], r["token"], r["classification"]))
    by_token = Counter(r["token"] for r in records)
    by_class = Counter(r["classification"] for r in records)
    raw_gate_refs = [
        r
        for r in records
        if r["classification"] == "executable_or_source_reference"
        and r["token"] in GATE_TOKENS
    ]
    raw_call_sites = [r for r in raw_gate_refs if r["call_like"]]

    r2_records = project_r2(records)
    r2_gate_records = project_r2(raw_gate_refs)
    r2_call_records = project_r2(raw_call_sites)
    record_set_sha = canonical_sha256(r2_records)
    gate_set_sha = canonical_sha256(r2_gate_records)
    call_set_sha = canonical_sha256(r2_call_records)

    # Exact R2 occurrence-set theorem. Aggregate checks remain for diagnostics,
    # but these three hashes prevent a different corpus from passing merely by
    # reproducing the same totals.
    if len(records) != EXPECTED_TOTAL_MATCHES:
        raise SystemExit(f"R2 total mismatch: {len(records)} != {EXPECTED_TOTAL_MATCHES}")
    if record_set_sha != EXPECTED_RECORD_SET_SHA256:
        raise SystemExit(f"R2 exact record-set mismatch: {record_set_sha}")
    if dict(sorted(by_token.items())) != EXPECTED_TOKEN_COUNTS:
        raise SystemExit("R2 token census mismatch: " + json.dumps(dict(sorted(by_token.items())), sort_keys=True))
    if dict(sorted(by_class.items())) != EXPECTED_CLASS_COUNTS:
        raise SystemExit("R2 classification census mismatch: " + json.dumps(dict(sorted(by_class.items())), sort_keys=True))
    if len(raw_gate_refs) != EXPECTED_EXECUTABLE_GATE_REFERENCES:
        raise SystemExit(f"R2 executable gate mismatch: {len(raw_gate_refs)} != {EXPECTED_EXECUTABLE_GATE_REFERENCES}")
    if gate_set_sha != EXPECTED_GATE_SET_SHA256:
        raise SystemExit(f"R2 exact gate-set mismatch: {gate_set_sha}")
    if len(raw_call_sites) != EXPECTED_CALL_LIKE_SITES:
        raise SystemExit(f"R2 call-like mismatch: {len(raw_call_sites)} != {EXPECTED_CALL_LIKE_SITES}")
    if call_set_sha != EXPECTED_CALL_SET_SHA256:
        raise SystemExit(f"R2 exact call-set mismatch: {call_set_sha}")

    unique_gate_refs = dedupe(raw_gate_refs)
    unique_call_sites = dedupe(raw_call_sites)
    alias_gate_groups = [r for r in unique_gate_refs if r["path_alias_count"] > 1]
    alias_call_groups = [r for r in unique_call_sites if r["path_alias_count"] > 1]

    payload = {
        "schema": "mycelix.finance.fin-safe-028.governance-gate-census.v0.5",
        "source_sha": SOURCE_SHA,
        "r2_population_binding": {
            "schema": "mycelix.finance.fin-safe-028.governance-gate-census.v0.1",
            "run": 35074110566,
            "artifact_id": 10441301626,
            "canonical_payload_sha256": "664e76200d860fefa7db7e30eeccdcd1a281af655aebdab944974e68c7ad49a5",
            "total_matches": EXPECTED_TOTAL_MATCHES,
            "executable_gate_references": EXPECTED_EXECUTABLE_GATE_REFERENCES,
            "call_like_sites": EXPECTED_CALL_LIKE_SITES,
            "exact_record_set_sha256": EXPECTED_RECORD_SET_SHA256,
            "exact_gate_set_sha256": EXPECTED_GATE_SET_SHA256,
            "exact_call_set_sha256": EXPECTED_CALL_SET_SHA256,
            "counts_by_token": EXPECTED_TOKEN_COUNTS,
            "counts_by_classification": EXPECTED_CLASS_COUNTS,
        },
        "identity_rule": "git_blob_sha+line+token",
        "regular_byte_rule": "worktree bytes accepted only when git hash-object --stdin equals stage-0 index blob SHA",
        "nonregular_rule": "symlink/gitlink entries are topology records, never text-scanned; exact R2 occurrence hashes prove exclusion preserves the qualified corpus",
        "repository_topology": {"nonregular_entries": topology},
        "regular_files_verified": regular_files_verified,
        "utf8_files_scanned": utf8_files_scanned,
        "tokens": list(TOKENS),
        "raw_total_matches": len(records),
        "raw_executable_gate_references": len(raw_gate_refs),
        "unique_executable_gate_references": len(unique_gate_refs),
        "raw_call_like_sites": len(raw_call_sites),
        "unique_call_like_sites": len(unique_call_sites),
        "mirrored_gate_identity_groups": len(alias_gate_groups),
        "mirrored_call_identity_groups": len(alias_call_groups),
        "raw_counts_by_token": dict(sorted(by_token.items())),
        "raw_counts_by_classification": dict(sorted(by_class.items())),
        "unique_gate_counts_by_token": dict(sorted(Counter(r["token"] for r in unique_gate_refs).items())),
        "unique_call_counts_by_token": dict(sorted(Counter(r["token"] for r in unique_call_sites).items())),
        "unique_gate_references": unique_gate_refs,
        "unique_call_like_sites_detail": unique_call_sites,
    }

    raw = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode()
    json_path = pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3d.json")
    json_path.write_bytes(raw)
    digest = hashlib.sha256(raw).hexdigest()
    pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3d.sha256").write_text(
        f"{digest}  fin-safe-028-governance-gate-census-r3d.json\n"
    )

    lines = [
        "# FIN-SAFE-028 governance gate census R3d",
        "",
        f"source_sha: `{SOURCE_SHA}`",
        "r2_exact_occurrence_binding: PASS",
        f"record_set_sha256: `{record_set_sha}`",
        f"gate_set_sha256: `{gate_set_sha}`",
        f"call_set_sha256: `{call_set_sha}`",
        f"regular_files_verified: {regular_files_verified}",
        f"utf8_files_scanned: {utf8_files_scanned}",
        f"nonregular_entries: {len(topology)}",
        f"raw_total_matches: {len(records)}",
        f"raw_executable_gate_references: {len(raw_gate_refs)}",
        f"unique_executable_gate_references: {len(unique_gate_refs)}",
        f"raw_call_like_sites: {len(raw_call_sites)}",
        f"unique_call_like_sites: {len(unique_call_sites)}",
        f"mirrored_gate_identity_groups: {len(alias_gate_groups)}",
        f"mirrored_call_identity_groups: {len(alias_call_groups)}",
        f"sha256: `{digest}`",
        "",
        "## Non-regular repository topology",
    ]
    for row in topology:
        lines.append(f"- `{row['kind']}` `{row['path']}` mode `{row['mode']}` object `{row['object_sha']}`")
    lines.extend(["", "## Unique call-like sites"])
    for row in unique_call_sites:
        aliases = ", ".join(row["paths"])
        lines.append(
            f"- `{row['token']}` blob `{row['blob_sha']}` line {row['line']} "
            f"aliases={row['path_alias_count']} paths=[{aliases}] — `{row['text']}`"
        )
    pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3d.md").write_text("\n".join(lines) + "\n")

    print(json.dumps({
        "source_sha": SOURCE_SHA,
        "r2_exact_occurrence_binding": "PASS",
        "record_set_sha256": record_set_sha,
        "gate_set_sha256": gate_set_sha,
        "call_set_sha256": call_set_sha,
        "regular_files_verified": regular_files_verified,
        "utf8_files_scanned": utf8_files_scanned,
        "nonregular_entries": len(topology),
        "raw_total_matches": len(records),
        "raw_executable_gate_references": len(raw_gate_refs),
        "unique_executable_gate_references": len(unique_gate_refs),
        "raw_call_like_sites": len(raw_call_sites),
        "unique_call_like_sites": len(unique_call_sites),
        "mirrored_gate_identity_groups": len(alias_gate_groups),
        "mirrored_call_identity_groups": len(alias_call_groups),
        "sha256": digest,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
