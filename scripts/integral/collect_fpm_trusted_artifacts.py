#!/usr/bin/env python3
"""Bounded, fail-closed GitHub Actions artifact enumerator for FPM evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any, Callable

PAGE_SIZE = 100
MAX_ARTIFACTS = 256
MAX_PAGES = 4
SCHEMA = "mycelix.fpm.trusted-qualification-artifact-enumeration.v1"


def fail(message: str) -> None:
    raise SystemExit(f"FPM_ARTIFACT_ENUMERATION_FAIL: {message}")


def canonical(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode(
        "utf-8"
    )


def page_items(raw: bytes, page: int) -> tuple[int, list[dict[str, Any]]]:
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"page {page} is not valid JSON: {exc}")
    if not isinstance(value, dict):
        fail(f"page {page} is not a JSON object")
    total = value.get("total_count")
    items = value.get("artifacts")
    if type(total) is not int or total < 0:
        fail(f"page {page} has invalid total_count")
    if not isinstance(items, list):
        fail(f"page {page} has invalid artifacts list")
    if len(items) > PAGE_SIZE:
        fail(f"page {page} exceeds fixed page size {PAGE_SIZE}")
    return total, items


def load_live_page(repository: str, run_id: int, page: int) -> bytes:
    try:
        proc = subprocess.run(
            [
                "gh",
                "api",
                f"repos/{repository}/actions/runs/{run_id}/artifacts?per_page={PAGE_SIZE}&page={page}&direction=asc",
            ],
            check=False,
            capture_output=True,
        )
    except OSError as exc:
        fail(f"unable to invoke gh: {exc}")
    if proc.returncode != 0:
        fail(f"artifact API request for page {page} failed: {proc.stderr.decode(errors='replace')}")
    return proc.stdout


def load_fixture_page(fixtures: Path, page: int) -> bytes:
    path = fixtures / f"page-{page}.json"
    if not path.is_file():
        return b'{"total_count":0,"artifacts":[]}'
    return path.read_bytes()


def identity_commitment(items: list[dict[str, Any]]) -> str:
    identities = []
    for item in items:
        if type(item) is not dict:
            fail("artifact item is not an object")
        artifact_id = item.get("id")
        name = item.get("name")
        if type(artifact_id) is not int or artifact_id <= 0:
            fail("artifact has invalid positive integer id")
        if not isinstance(name, str) or not name:
            fail("artifact has empty/non-string name")
        identities.append({"id": artifact_id, "name": name})
    return hashlib.sha256(canonical(identities)).hexdigest()


def enumerate_artifacts(
    get_page: Callable[[int], tuple[int, list[dict[str, Any]]]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    items: list[dict[str, Any]] = []
    seen_ids: set[int] = set()
    seen_names: set[str] = set()
    page_counts: list[int] = []
    total_count: int | None = None

    for page in range(1, MAX_PAGES + 1):
        reported_total, page_items_value = get_page(page)
        if total_count is None:
            total_count = reported_total
        elif reported_total != total_count:
            fail(
                f"total_count changed across pages: {total_count} -> {reported_total}"
            )
        page_counts.append(len(page_items_value))

        for item in page_items_value:
            if type(item) is not dict:
                fail(f"page {page} contains a non-object artifact")
            artifact_id = item.get("id")
            name = item.get("name")
            if type(artifact_id) is not int or artifact_id <= 0:
                fail(f"page {page} contains invalid artifact id")
            if not isinstance(name, str) or not name:
                fail(f"page {page} contains invalid artifact name")
            if artifact_id in seen_ids:
                fail(f"duplicate artifact id across pages: {artifact_id}")
            if name in seen_names:
                fail(f"duplicate artifact name across pages: {name}")
            seen_ids.add(artifact_id)
            seen_names.add(name)
            items.append(item)
            if len(items) > MAX_ARTIFACTS:
                fail(f"artifact count exceeds hard bound {MAX_ARTIFACTS}")

        if len(items) > MAX_ARTIFACTS:
            fail(f"artifact count exceeds hard bound {MAX_ARTIFACTS}")

        if len(page_items_value) < PAGE_SIZE:
            terminal_page = page
            break
    else:
        fail(f"artifact enumeration exceeded maximum page bound {MAX_PAGES}")

    assert total_count is not None
    if total_count > MAX_ARTIFACTS:
        fail(f"API total_count {total_count} exceeds hard bound {MAX_ARTIFACTS}")
    if total_count != len(items):
        fail(
            f"artifact enumeration incomplete: API total_count={total_count}, "
            f"enumerated={len(items)}"
        )

    metadata = {
        "schema": SCHEMA,
        "page_size": PAGE_SIZE,
        "max_pages": MAX_PAGES,
        "max_artifacts": MAX_ARTIFACTS,
        "total_count_reported": total_count,
        "enumerated_count": len(items),
        "page_counts": page_counts,
        "terminal_page": terminal_page,
        "artifact_identity_sha256": identity_commitment(items),
        "complete": True,
    }
    return items, metadata


def enumerate_consistent_artifacts(
    get_page: Callable[[int], tuple[int, list[dict[str, Any]]]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    artifacts, metadata = enumerate_artifacts(get_page)
    second_artifacts, second_metadata = enumerate_artifacts(get_page)
    if second_metadata["total_count_reported"] != metadata["total_count_reported"]:
        fail("artifact total_count changed between complete enumeration passes")
    if second_metadata["page_counts"] != metadata["page_counts"]:
        fail("artifact page counts changed between complete enumeration passes")
    if second_metadata["artifact_identity_sha256"] != metadata["artifact_identity_sha256"]:
        fail("artifact identity sequence changed between complete enumeration passes")
    if second_artifacts != artifacts:
        fail("artifact payload changed between complete enumeration passes")
    metadata["repeat_enumeration_verified"] = True
    metadata["repeat_total_count_reported"] = second_metadata["total_count_reported"]
    metadata["repeat_page_counts"] = second_metadata["page_counts"]
    metadata["repeat_artifact_identity_sha256"] = second_metadata["artifact_identity_sha256"]
    return artifacts, metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", default=None)
    parser.add_argument("--run-id", type=int, default=None)
    parser.add_argument("--fixtures-dir", type=Path, default=None)
    parser.add_argument("--artifacts-out", type=Path, required=True)
    parser.add_argument("--enumeration-out", type=Path, required=True)
    args = parser.parse_args()

    if args.fixtures_dir is not None:
        if args.repository is not None or args.run_id is not None:
            fail("fixtures mode cannot be combined with live API identity arguments")

        def get_page(page: int) -> tuple[int, list[dict[str, Any]]]:
            return page_items(load_fixture_page(args.fixtures_dir, page), page)
    else:
        if not args.repository or args.run_id is None or args.run_id < 1:
            fail("live mode requires --repository and a positive --run-id")

        def get_page(page: int) -> tuple[int, list[dict[str, Any]]]:
            return page_items(load_live_page(args.repository, args.run_id, page), page)

    artifacts, metadata = enumerate_consistent_artifacts(get_page)
    args.artifacts_out.parent.mkdir(parents=True, exist_ok=True)
    args.enumeration_out.parent.mkdir(parents=True, exist_ok=True)
    args.artifacts_out.write_bytes(canonical({"artifacts": artifacts}) + b"\n")
    args.enumeration_out.write_bytes(canonical(metadata) + b"\n")
    print(json.dumps(metadata, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
