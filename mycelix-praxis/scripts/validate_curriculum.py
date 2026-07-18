#!/usr/bin/env python3
"""Validate the active Praxis curriculum and generate its lesson manifest.

The active graph is derived from the same ``include_str!`` declarations used
by the Leptos application. This keeps build-time validation and runtime content
selection on one source of truth.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
CURRICULUM_RS = ROOT / "apps/leptos/src/curriculum.rs"
LESSON_ROOT = ROOT / "examples/curriculum/caps/generated"
MANIFEST_PATH = ROOT / "apps/leptos/static/content-manifest.json"
INCLUDE_RE = re.compile(r'include_str!\(\s*"([^"]+\.json)"\s*\)')


def load_json(path: Path, errors: list[str]) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        errors.append(f"{path.relative_to(ROOT)}: invalid JSON: {error}")
        return None
    if not isinstance(value, dict):
        errors.append(f"{path.relative_to(ROOT)}: top-level JSON value must be an object")
        return None
    return value


def active_documents(errors: list[str]) -> list[tuple[Path, dict[str, Any]]]:
    source = CURRICULUM_RS.read_text(encoding="utf-8")
    relative_paths = INCLUDE_RE.findall(source)
    if not relative_paths:
        errors.append("apps/leptos/src/curriculum.rs: no embedded curriculum documents found")
        return []

    documents: list[tuple[Path, dict[str, Any]]] = []
    seen_paths: set[Path] = set()
    for relative_path in relative_paths:
        path = (CURRICULUM_RS.parent / relative_path).resolve()
        if ROOT not in path.parents:
            errors.append(f"curriculum include escapes repository root: {relative_path}")
            continue
        if path in seen_paths:
            errors.append(f"curriculum document included more than once: {path.relative_to(ROOT)}")
            continue
        seen_paths.add(path)
        document = load_json(path, errors)
        if document is not None:
            documents.append((path, document))
    return documents


def validate_graph(
    documents: list[tuple[Path, dict[str, Any]]], errors: list[str]
) -> tuple[set[str], int]:
    node_sources: dict[str, list[Path]] = defaultdict(list)
    edges: list[tuple[Path, dict[str, Any]]] = []

    for path, document in documents:
        nodes = document.get("nodes", [])
        document_edges = document.get("edges", [])
        if not isinstance(nodes, list) or not isinstance(document_edges, list):
            errors.append(f"{path.relative_to(ROOT)}: nodes and edges must be arrays")
            continue
        for index, node in enumerate(nodes):
            node_id = node.get("id") if isinstance(node, dict) else None
            if not isinstance(node_id, str) or not node_id.strip():
                errors.append(
                    f"{path.relative_to(ROOT)}: node {index} has no non-empty string id"
                )
                continue
            node_sources[node_id].append(path)
        for edge in document_edges:
            if isinstance(edge, dict):
                edges.append((path, edge))
            else:
                errors.append(f"{path.relative_to(ROOT)}: edge must be an object")

    for node_id, sources in sorted(node_sources.items()):
        if len(sources) > 1:
            locations = ", ".join(str(path.relative_to(ROOT)) for path in sources)
            errors.append(f"duplicate node id {node_id!r}: {locations}")

    node_ids = set(node_sources)
    for path, edge in edges:
        source = edge.get("from")
        target = edge.get("to")
        if not isinstance(source, str) or not isinstance(target, str):
            errors.append(f"{path.relative_to(ROOT)}: edge requires string from/to ids")
            continue
        missing = [node_id for node_id in (source, target) if node_id not in node_ids]
        if missing:
            errors.append(
                f"{path.relative_to(ROOT)}: dangling edge {source!r} -> {target!r}; "
                f"missing {', '.join(repr(node_id) for node_id in missing)}"
            )

    return node_ids, len(edges)


def build_lesson_manifest(errors: list[str]) -> dict[str, Any]:
    lessons: dict[str, list[dict[str, str]]] = defaultdict(list)

    for path in sorted(LESSON_ROOT.rglob("*.json")):
        document = load_json(path, errors)
        if document is None:
            continue
        has_node = "node_id" in document
        has_lesson = "lesson" in document
        if not has_node and not has_lesson:
            # Generator progress files are valid metadata, not lesson assets.
            continue
        if not has_node or not has_lesson:
            errors.append(
                f"{path.relative_to(ROOT)}: generated content must contain both node_id and lesson"
            )
            continue

        node_id = document.get("node_id")
        lesson = document.get("lesson")
        if not isinstance(node_id, str) or not node_id.strip() or not isinstance(lesson, dict):
            errors.append(f"{path.relative_to(ROOT)}: invalid node_id or lesson object")
            continue

        title = lesson.get("title", "")
        if not isinstance(title, str):
            errors.append(f"{path.relative_to(ROOT)}: lesson title must be a string")
            continue

        relative_asset = path.relative_to(LESSON_ROOT).as_posix()
        lessons[node_id].append(
            {
                "path": f"/curriculum/generated/{relative_asset}",
                "title": title,
            }
        )

    return {
        "schema_version": 1,
        "lessons": {
            node_id: sorted(assets, key=lambda asset: asset["path"])
            for node_id, assets in sorted(lessons.items())
        },
    }


def encoded_manifest(manifest: dict[str, Any]) -> str:
    return json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write",
        action="store_true",
        help="write the deterministic content manifest instead of checking it",
    )
    args = parser.parse_args()

    errors: list[str] = []
    documents = active_documents(errors)
    node_ids, edge_count = validate_graph(documents, errors)
    manifest = build_lesson_manifest(errors)
    expected = encoded_manifest(manifest)

    if args.write and not errors:
        MANIFEST_PATH.write_text(expected, encoding="utf-8")
    elif not args.write:
        if not MANIFEST_PATH.exists():
            errors.append(
                "apps/leptos/static/content-manifest.json is missing; "
                "run scripts/validate_curriculum.py --write"
            )
        elif MANIFEST_PATH.read_text(encoding="utf-8") != expected:
            errors.append(
                "apps/leptos/static/content-manifest.json is stale; "
                "run scripts/validate_curriculum.py --write"
            )

    if errors:
        print("Curriculum validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    lessons = manifest["lessons"]
    lesson_assets = sum(len(assets) for assets in lessons.values())
    active_lesson_nodes = len(node_ids.intersection(lessons))
    print(
        "Curriculum valid: "
        f"{len(node_ids)} nodes, {edge_count} edges, "
        f"{lesson_assets} lesson assets for {len(lessons)} ids, "
        f"{active_lesson_nodes} active graph nodes with generated lessons."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
