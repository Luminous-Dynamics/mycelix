#!/usr/bin/env python3
"""Fetch and safely extract the exact D6U trusted artifact."""

import hashlib
import json
import os
import pathlib
import stat
import struct
import sys
import urllib.parse
import urllib.request
from zipfile import BadZipFile, ZipFile



if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")

ROOT = pathlib.Path(__file__).parents[2]
POLICY = ROOT / "docs/integral/d6u-trusted-builder-policy.json"

MAX_GITHUB_JSON_BYTES = 8 * 1024 * 1024

API_HEADERS = {
    "Accept": "application/vnd.github+json",
    "X-GitHub-Api-Version": "2026-03-10",
    "User-Agent": "mycelix-d6u-trusted-attestation",
}

EXPECTED_FILES = {
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
}

HANDOFF_EXPECTED_FILES = {
    "Cargo.lock",
    "d6u-auditor-context.txt",
    "d6u-auditor-handoff.manifest.sha256",
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "d6u-trusted-evidence-predicate.json",
}

MAX_ZIP_EOCD_SEARCH_BYTES = 22 + 65535
ALLOWED_ZIP_COMPRESSION_METHODS = {0, 8}  # stored, deflate (Zlib)


def github_get(repo: str, api_path: str, token: str) -> dict:
    request = urllib.request.Request(
        f"https://api.github.com/repos/{repo}{api_path}",
        headers={**API_HEADERS, "Authorization": f"Bearer {token}"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        payload = response.read(MAX_GITHUB_JSON_BYTES + 1)
        if len(payload) > MAX_GITHUB_JSON_BYTES:
            raise RuntimeError(
                f"GitHub API response exceeded {MAX_GITHUB_JSON_BYTES} bytes"
            )
        return json.loads(payload)


def expected_artifact(repo: str, event: dict, policy: dict) -> dict:
    workflow_run = event["workflow_run"]
    event_repo = event["repository"]
    assert event_repo["full_name"] == repo
    assert workflow_run["event"] == "workflow_run"
    assert workflow_run["name"] == policy["workflow_name"]
    assert workflow_run["path"] == policy["workflow_path"]
    assert workflow_run["conclusion"] == "success"
    assert workflow_run["repository"]["full_name"] == repo
    assert workflow_run["head_repository"]["full_name"] == repo
    assert workflow_run["head_branch"] == policy["source_branch"]

    run_id = workflow_run["id"]
    run_attempt = workflow_run["run_attempt"]
    expected_name = (
        f"d6u-runtime-evidence-run-{run_id}-attempt-{run_attempt}"
    )
    query = urllib.parse.urlencode({"name": expected_name})
    token = os.environ["GITHUB_TOKEN"]
    current_run = github_get(
        repo,
        f"/actions/runs/{run_id}",
        token,
    )
    assert current_run["id"] == run_id
    assert current_run["run_attempt"] == run_attempt
    assert current_run["repository"]["full_name"] == repo
    assert current_run["head_branch"] == os.environ["D6U_TRIGGER_HEAD_BRANCH"]
    assert current_run["head_sha"] == os.environ["D6U_TRIGGER_HEAD_SHA"]
    payload = github_get(
        repo,
        f"/actions/runs/{run_id}/artifacts?{query}",
        token,
    )
    artifacts = payload.get("artifacts", [])
    assert len(artifacts) == 1, (
        f"expected exactly one trusted artifact, observed {len(artifacts)}"
    )
    artifact = artifacts[0]
    assert artifact["name"] == expected_name
    assert artifact["expired"] is False
    workflow_artifact_run = artifact["workflow_run"]
    assert workflow_artifact_run["id"] == run_id
    assert workflow_artifact_run["repository_id"] == event["repository"]["id"]
    assert workflow_artifact_run["head_repository_id"] == event["repository"]["id"]
    assert workflow_artifact_run["head_branch"] == workflow_run["head_branch"]
    assert workflow_artifact_run["head_sha"] == workflow_run["head_sha"]

    digest = artifact.get("digest", "")
    assert digest.startswith("sha256:") and len(digest) == 71, (
        f"missing or malformed GitHub artifact digest: {digest!r}"
    )
    maximum = int(policy["artifact_max_total_bytes"])
    assert artifact["size_in_bytes"] <= maximum, (
        f"artifact archive exceeds trusted maximum: "
        f"{artifact['size_in_bytes']} > {maximum}"
    )
    return artifact


def expected_current_run_artifact(repo: str, policy: dict) -> dict:
    run_id = int(os.environ["GITHUB_RUN_ID"])
    run_attempt = int(os.environ["GITHUB_RUN_ATTEMPT"])
    token = os.environ["GITHUB_TOKEN"]
    expected_name = policy["auditor_handoff"]["artifact_name_template"].format(
        run_id=run_id,
        run_attempt=run_attempt,
    )

    current_run = github_get(
        repo,
        f"/actions/runs/{run_id}",
        token,
    )
    assert current_run["id"] == run_id
    assert current_run["run_attempt"] == run_attempt
    assert current_run["repository"]["full_name"] == repo
    assert current_run["head_repository"]["full_name"] == repo
    expected_ref = f"refs/heads/{current_run['head_branch']}"
    assert os.environ["GITHUB_REF"] == expected_ref
    assert current_run["head_sha"] == os.environ["GITHUB_SHA"]

    query = urllib.parse.urlencode({"name": expected_name})
    payload = github_get(
        repo,
        f"/actions/runs/{run_id}/artifacts?{query}",
        token,
    )
    artifacts = payload.get("artifacts", [])
    assert len(artifacts) == 1, (
        f"expected exactly one current-run auditor handoff artifact, observed {len(artifacts)}"
    )
    artifact = artifacts[0]
    assert artifact["name"] == expected_name
    assert artifact["expired"] is False
    workflow_artifact_run = artifact["workflow_run"]
    assert workflow_artifact_run["id"] == run_id
    assert workflow_artifact_run["repository_id"] == current_run["repository"]["id"]
    assert workflow_artifact_run["head_repository_id"] == current_run["head_repository"]["id"]
    assert workflow_artifact_run["head_branch"] == current_run["head_branch"]
    assert workflow_artifact_run["head_sha"] == current_run["head_sha"]
    digest = artifact.get("digest", "")
    assert digest.startswith("sha256:") and len(digest) == 71, (
        f"missing or malformed GitHub artifact digest: {digest!r}"
    )
    maximum = int(policy["auditor_handoff"]["artifact_max_archive_bytes"])
    assert artifact["size_in_bytes"] <= maximum, (
        f"auditor handoff archive exceeds trusted maximum: "
        f"{artifact['size_in_bytes']} > {maximum}"
    )
    return artifact


def download_archive(repo: str, artifact_id: int, expected_digest: str, destination: pathlib.Path, maximum: int) -> None:
    request = urllib.request.Request(
        f"https://api.github.com/repos/{repo}/actions/artifacts/{artifact_id}/zip",
        headers={**API_HEADERS, "Authorization": f"Bearer {os.environ['GITHUB_TOKEN']}"},
    )
    observed = hashlib.sha256()
    written = 0
    with urllib.request.urlopen(request, timeout=120) as response, destination.open("wb") as output:
        while chunk := response.read(1024 * 1024):
            written += len(chunk)
            assert written <= maximum, (
                f"downloaded artifact archive exceeds trusted maximum: "
                f"{written} > {maximum}"
            )
            observed.update(chunk)
            output.write(chunk)
    observed_digest = "sha256:" + observed.hexdigest()
    assert observed_digest == expected_digest, (
        f"artifact archive digest mismatch: "
        f"expected={expected_digest}, observed={observed_digest}"
    )


def is_symlink_member(info) -> bool:
    mode = (info.external_attr >> 16) & 0xFFFF
    return stat.S_ISLNK(mode)


def preflight_zip_entry_count(archive_path: pathlib.Path, maximum_entries: int) -> None:
    archive_size = archive_path.stat().st_size
    assert archive_size >= 22, "trusted artifact ZIP is smaller than an EOCD record"
    with archive_path.open("rb") as handle:
        handle.seek(max(0, archive_size - MAX_ZIP_EOCD_SEARCH_BYTES))
        tail = handle.read(MAX_ZIP_EOCD_SEARCH_BYTES)

    marker = b"PK\x05\x06"
    end = -1
    cursor = len(tail)
    while True:
        position = tail.rfind(marker, 0, cursor)
        if position < 0:
            break
        if position + 22 <= len(tail):
            comment_length = struct.unpack_from("<H", tail, position + 20)[0]
            if position + 22 + comment_length == len(tail):
                end = position
                break
        cursor = position

    assert end >= 0, "trusted artifact ZIP has no valid EOCD record"
    disk_number = struct.unpack_from("<H", tail, end + 4)[0]
    central_directory_disk = struct.unpack_from("<H", tail, end + 6)[0]
    entries_on_disk = struct.unpack_from("<H", tail, end + 8)[0]
    total_entries = struct.unpack_from("<H", tail, end + 10)[0]
    assert disk_number == 0 and central_directory_disk == 0, (
        "trusted artifact ZIP uses a multi-disk layout"
    )
    assert entries_on_disk == total_entries, (
        "trusted artifact ZIP has inconsistent entry counts"
    )
    assert total_entries != 0xFFFF, (
        "trusted artifact ZIP Zip64 entry counts are not permitted"
    )
    assert total_entries <= maximum_entries, (
        f"trusted artifact ZIP entry count exceeds trusted maximum: "
        f"{total_entries} > {maximum_entries}"
    )


def verify_zip_members(
    archive_path: pathlib.Path,
    policy: dict,
    expected_files: set[str] | None = None,
    maximums: dict[str, int] | None = None,
    maximum_entries: int | None = None,
    maximum_total: int | None = None,
    allowed_compression_methods: list[str] | None = None,
) -> list:
    expected = set(EXPECTED_FILES if expected_files is None else expected_files)
    maximums = policy["artifact_max_bytes"] if maximums is None else maximums
    maximum_entries = (
        int(policy["artifact_max_entries"])
        if maximum_entries is None
        else int(maximum_entries)
    )
    maximum_total = (
        int(policy["artifact_max_total_bytes"])
        if maximum_total is None
        else int(maximum_total)
    )

    preflight_zip_entry_count(archive_path, maximum_entries)

    try:
        archive = ZipFile(archive_path)
    except BadZipFile as exc:
        raise AssertionError("trusted artifact is not a valid ZIP archive") from exc

    with archive:
        infos = archive.infolist()
        assert len(infos) <= maximum_entries
        assert len(infos) == len(expected), (
            f"trusted artifact member count mismatch: "
            f"expected={len(expected)}, observed={len(infos)}"
        )
        names = [info.filename for info in infos]
        assert len(set(names)) == len(names), "trusted artifact contains duplicate ZIP members"
        assert set(names) == expected, (
            f"trusted artifact ZIP members mismatch: observed={sorted(names)!r}"
        )

        total_uncompressed = 0
        for info in infos:
            assert not info.is_dir(), f"trusted artifact contains directory member: {info.filename!r}"
            assert not is_symlink_member(info), (
                f"trusted artifact contains symlink member: {info.filename!r}"
            )
            assert not (info.flag_bits & 0x1), (
                f"trusted artifact contains encrypted member: {info.filename!r}"
            )
            compression_codes = {"stored": 0, "deflate": 8}
            configured_compression_methods = (
                policy["artifact_integrity"]["allowed_compression_methods"]
                if allowed_compression_methods is None
                else allowed_compression_methods
            )
            assert set(configured_compression_methods) <= set(compression_codes)
            allowed_compression_codes = {
                compression_codes[name] for name in configured_compression_methods
            }
            assert info.compress_type in allowed_compression_codes, (
                f"trusted artifact contains unsupported ZIP compression method: "
                f"{info.filename!r}: {info.compress_type}"
            )
            maximum = int(maximums[info.filename])
            assert info.file_size <= maximum, (
                f"trusted artifact member is too large: "
                f"{info.filename!r}: {info.file_size} > {maximum}"
            )
            total_uncompressed += info.file_size

        assert total_uncompressed <= maximum_total, (
            f"trusted artifact uncompressed size is too large: "
            f"{total_uncompressed} > {maximum_total}"
        )
        return infos


def extract_members(
    archive_path: pathlib.Path,
    destination: pathlib.Path,
    infos: list,
    policy: dict,
    maximums: dict[str, int] | None = None,
) -> None:
    maximums = policy["artifact_max_bytes"] if maximums is None else maximums
    destination.mkdir(parents=True, exist_ok=True)

    with ZipFile(archive_path) as archive:
        for info in infos:
            target = destination / info.filename
            assert target.parent == destination, (
                f"trusted artifact member is not a root file: {info.filename!r}"
            )
            with archive.open(info, "r") as source, target.open("xb") as output:
                copied = 0
                maximum = int(maximums[info.filename])
                while chunk := source.read(min(1024 * 1024, maximum - copied + 1)):
                    copied += len(chunk)
                    assert copied <= maximum, (
                        f"trusted artifact member exceeded extraction bound: "
                        f"{info.filename!r}"
                    )
                    output.write(chunk)
                assert copied == info.file_size, (
                    f"trusted artifact member size mismatch: "
                    f"{info.filename!r}: expected={info.file_size}, observed={copied}"
                )


def main() -> None:
    assert len(sys.argv) in {2, 3}, (
        "usage: fetch_d6u_trusted_artifact.py DESTINATION_DIR | "
        "--current-run-handoff DESTINATION_DIR"
    )
    current_run_handoff = len(sys.argv) == 3
    if current_run_handoff:
        assert sys.argv[1] == "--current-run-handoff"
        destination = pathlib.Path(sys.argv[2]).resolve()
    else:
        destination = pathlib.Path(sys.argv[1]).resolve()

    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    repo = os.environ["GITHUB_REPOSITORY"]

    if current_run_handoff:
        expected_workflow_ref = (
            f"{repo}/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main"
        )
        assert os.environ["GITHUB_WORKFLOW_REF"] == expected_workflow_ref
        artifact = expected_current_run_artifact(repo, policy)
        maximums = {
            name: int(value)
            for name, value in policy["auditor_handoff"]["artifact_max_bytes"].items()
        }
        expected_files = HANDOFF_EXPECTED_FILES
        maximum_entries = int(policy["auditor_handoff"]["artifact_max_entries"])
        maximum_total = int(policy["auditor_handoff"]["artifact_max_total_bytes"])
        maximum_archive = int(policy["auditor_handoff"]["artifact_max_archive_bytes"])
    else:
        event = json.loads(
            pathlib.Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8")
        )
        artifact = expected_artifact(repo, event, policy)
        maximums = policy["artifact_max_bytes"]
        expected_files = EXPECTED_FILES
        maximum_entries = int(policy["artifact_max_entries"])
        maximum_total = int(policy["artifact_max_total_bytes"])
        maximum_archive = int(policy["artifact_max_total_bytes"])

    archive_path = pathlib.Path(os.environ["RUNNER_TEMP"]) / (
        "d6u-trusted-auditor-handoff.zip"
        if current_run_handoff
        else "d6u-trusted-artifact.zip"
    )
    try:
        download_archive(
            repo,
            int(artifact["id"]),
            artifact["digest"],
            archive_path,
            maximum_archive,
        )
        infos = verify_zip_members(
            archive_path,
            policy,
            expected_files=expected_files,
            maximums=maximums,
            maximum_entries=maximum_entries,
            maximum_total=maximum_total,
            allowed_compression_methods=(
                policy["auditor_handoff"]["allowed_compression_methods"]
                if current_run_handoff
                else policy["artifact_integrity"]["allowed_compression_methods"]
            ),
        )
        extract_members(
            archive_path,
            destination,
            infos,
            policy,
            maximums=maximums,
        )
        label = "current-run auditor handoff" if current_run_handoff else "D6U artifact"
        print(
            f"verified and safely extracted {label}: "
            f"id={artifact['id']} digest={artifact['digest']}"
        )
    finally:
        archive_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
