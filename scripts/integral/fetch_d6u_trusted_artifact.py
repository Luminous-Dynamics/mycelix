#!/usr/bin/env python3
"""Fetch and safely extract the exact D6U trusted artifact."""

import hashlib
import json
import os
import pathlib
import stat
import sys
import urllib.parse
import urllib.request
from zipfile import BadZipFile, ZipFile



if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")

ROOT = pathlib.Path(__file__).parents[2]
POLICY = ROOT / "docs/integral/d6u-trusted-builder-policy.json"

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


def github_get(repo: str, api_path: str, token: str) -> dict:
    request = urllib.request.Request(
        f"https://api.github.com/repos/{repo}{api_path}",
        headers={**API_HEADERS, "Authorization": f"Bearer {token}"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def expected_artifact(repo: str, event: dict, policy: dict) -> dict:
    workflow_run = event["workflow_run"]
    run_id = workflow_run["id"]
    run_attempt = workflow_run["run_attempt"]
    expected_name = (
        f"d6u-runtime-evidence-run-{run_id}-attempt-{run_attempt}"
    )
    query = urllib.parse.urlencode({"name": expected_name})
    payload = github_get(
        repo,
        f"/actions/runs/{run_id}/artifacts?{query}",
        os.environ["GITHUB_TOKEN"],
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
    assert workflow_artifact_run["head_branch"] == "main"
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


def verify_zip_members(archive_path: pathlib.Path, policy: dict) -> list:
    expected = set(EXPECTED_FILES)
    maximums = policy["artifact_max_bytes"]
    maximum_entries = int(policy["artifact_max_entries"])
    maximum_total = int(policy["artifact_max_total_bytes"])

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
) -> None:
    maximums = policy["artifact_max_bytes"]
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
    assert len(sys.argv) == 2, "usage: fetch_d6u_trusted_artifact.py DESTINATION_DIR"
    destination = pathlib.Path(sys.argv[1]).resolve()
    event = json.loads(
        pathlib.Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8")
    )
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    repo = os.environ["GITHUB_REPOSITORY"]

    artifact = expected_artifact(repo, event, policy)
    archive_path = pathlib.Path(os.environ["RUNNER_TEMP"]) / "d6u-trusted-artifact.zip"
    try:
        download_archive(
            repo,
            int(artifact["id"]),
            artifact["digest"],
            archive_path,
            int(policy["artifact_max_total_bytes"]),
        )
        infos = verify_zip_members(archive_path, policy)
        extract_members(archive_path, destination, infos, policy)
        print(
            "verified and safely extracted D6U artifact: "
            f"id={artifact['id']} digest={artifact['digest']}"
        )
    finally:
        archive_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
