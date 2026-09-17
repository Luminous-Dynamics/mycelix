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
REGULAR_MODES = {"100644", "100755"}


def run(*args: str) -> bytes:
    return subprocess.check_output(args)


def classify(path: pathlib.PurePosixPath, line: str, token: str) -> str:
    # Intentionally equivalent in semantics to the qualified R2 classifier so
    # R3c changes only identity projection/byte transport, not source population.
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
        mode, blob_sha, stage = meta.decode("ascii").split()
        if stage != "0":
            raise SystemExit(f"non-stage-0 index entry for {raw_path!r}: stage={stage}")
        path = raw_path.decode("utf-8", errors="surrogateescape")
        index[path] = {"mode": mode, "blob_sha": blob_sha}
    return index


def verified_worktree_text(path_text: str, mode: str, expected_blob_sha: str):
    # R3b used `git cat-file blob <sha>`, which can fail on a shallow checkout
    # even when checkout materialized the exact file. R3c instead treats the
    # worktree only as a byte transport: for every tracked regular file, it
    # recomputes Git's blob object ID from those bytes and requires equality
    # with the stage-0 index SHA before the bytes are eligible for measurement.
    if mode not in REGULAR_MODES:
        raise SystemExit(f"unsupported tracked mode {mode} for {path_text}")
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


def dedupe(items):
    grouped = defaultdict(list)
    representative = {}
    for record in items:
        # Git blob SHA commits to every byte in the file. Line + token then
        # identify the exact occurrence inside that immutable content.
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

    records = []
    index = read_index()
    verified_regular_files = 0
    utf8_files = 0
    for path_text, meta in sorted(index.items()):
        text = verified_worktree_text(path_text, meta["mode"], meta["blob_sha"])
        verified_regular_files += 1
        if text is None:
            continue
        utf8_files += 1
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
                        "mode": meta["mode"],
                        "blob_sha": meta["blob_sha"],
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

    # Cross-version population theorem: R3c may deduplicate identities, but it
    # must first reproduce the exact qualified R2 raw census.
    if len(records) != EXPECTED_TOTAL_MATCHES:
        raise SystemExit(f"R2 total mismatch: {len(records)} != {EXPECTED_TOTAL_MATCHES}")
    if dict(sorted(by_token.items())) != EXPECTED_TOKEN_COUNTS:
        raise SystemExit(
            "R2 token census mismatch: "
            + json.dumps(dict(sorted(by_token.items())), sort_keys=True)
        )
    if dict(sorted(by_class.items())) != EXPECTED_CLASS_COUNTS:
        raise SystemExit(
            "R2 classification census mismatch: "
            + json.dumps(dict(sorted(by_class.items())), sort_keys=True)
        )
    if len(raw_gate_refs) != EXPECTED_EXECUTABLE_GATE_REFERENCES:
        raise SystemExit(
            f"R2 executable gate mismatch: {len(raw_gate_refs)} != "
            f"{EXPECTED_EXECUTABLE_GATE_REFERENCES}"
        )

    unique_gate_refs = dedupe(raw_gate_refs)
    unique_call_sites = dedupe(raw_call_sites)
    alias_gate_groups = [r for r in unique_gate_refs if r["path_alias_count"] > 1]
    alias_call_groups = [r for r in unique_call_sites if r["path_alias_count"] > 1]

    payload = {
        "schema": "mycelix.finance.fin-safe-028.governance-gate-census.v0.4",
        "source_sha": SOURCE_SHA,
        "r2_population_binding": {
            "schema": "mycelix.finance.fin-safe-028.governance-gate-census.v0.1",
            "run": 35074110566,
            "artifact_id": 10441301626,
            "canonical_payload_sha256": "664e76200d860fefa7db7e30eeccdcd1a281af655aebdab944974e68c7ad49a5",
            "total_matches": EXPECTED_TOTAL_MATCHES,
            "executable_gate_references": EXPECTED_EXECUTABLE_GATE_REFERENCES,
            "counts_by_token": EXPECTED_TOKEN_COUNTS,
            "counts_by_classification": EXPECTED_CLASS_COUNTS,
        },
        "identity_rule": "git_blob_sha+line+token",
        "blob_read_rule": (
            "worktree regular-file bytes are accepted only when git hash-object --stdin "
            "equals the stage-0 index blob SHA"
        ),
        "verified_regular_files": verified_regular_files,
        "utf8_files_scanned": utf8_files,
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
        "unique_gate_counts_by_token": dict(
            sorted(Counter(r["token"] for r in unique_gate_refs).items())
        ),
        "unique_call_counts_by_token": dict(
            sorted(Counter(r["token"] for r in unique_call_sites).items())
        ),
        "unique_gate_references": unique_gate_refs,
        "unique_call_like_sites_detail": unique_call_sites,
    }

    raw = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode()
    json_path = pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3c.json")
    json_path.write_bytes(raw)
    digest = hashlib.sha256(raw).hexdigest()
    pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3c.sha256").write_text(
        f"{digest}  fin-safe-028-governance-gate-census-r3c.json\n"
    )

    lines = [
        "# FIN-SAFE-028 governance gate census R3c",
        "",
        f"source_sha: `{SOURCE_SHA}`",
        "r2_population_binding: PASS",
        f"verified_regular_files: {verified_regular_files}",
        f"utf8_files_scanned: {utf8_files}",
        f"raw_total_matches: {len(records)}",
        f"raw_executable_gate_references: {len(raw_gate_refs)}",
        f"unique_executable_gate_references: {len(unique_gate_refs)}",
        f"raw_call_like_sites: {len(raw_call_sites)}",
        f"unique_call_like_sites: {len(unique_call_sites)}",
        f"mirrored_gate_identity_groups: {len(alias_gate_groups)}",
        f"mirrored_call_identity_groups: {len(alias_call_groups)}",
        f"sha256: `{digest}`",
        "",
        "## Unique call-like sites",
    ]
    for row in unique_call_sites:
        aliases = ", ".join(row["paths"])
        lines.append(
            f"- `{row['token']}` blob `{row['blob_sha']}` line {row['line']} "
            f"aliases={row['path_alias_count']} paths=[{aliases}] — `{row['text']}`"
        )
    pathlib.Path("/tmp/fin-safe-028-governance-gate-census-r3c.md").write_text(
        "\n".join(lines) + "\n"
    )

    print(
        json.dumps(
            {
                "source_sha": SOURCE_SHA,
                "r2_population_binding": "PASS",
                "verified_regular_files": verified_regular_files,
                "utf8_files_scanned": utf8_files,
                "raw_total_matches": len(records),
                "raw_executable_gate_references": len(raw_gate_refs),
                "unique_executable_gate_references": len(unique_gate_refs),
                "raw_call_like_sites": len(raw_call_sites),
                "unique_call_like_sites": len(unique_call_sites),
                "mirrored_gate_identity_groups": len(alias_gate_groups),
                "mirrored_call_identity_groups": len(alias_call_groups),
                "sha256": digest,
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
