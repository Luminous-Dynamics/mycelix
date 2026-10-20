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



def validate_essential_capabilities(errors: list[str]) -> tuple[int, int]:
    """Validate the versioned essential-capabilities seed and its active lessons.

    This intentionally validates the seed's local schema and does not claim that
    its content is pedagogically effective or suitable for every jurisdiction.
    """
    seed_path = ROOT / "examples/curriculum/essential_capabilities_v1.json"
    lesson_root = LESSON_ROOT / "essential-capabilities"
    manifest = load_json(MANIFEST_PATH, errors)

    seed = load_json(seed_path, errors)
    if seed is None:
        return 0, 0

    metadata = seed.get("metadata")
    nodes = seed.get("nodes")
    edges = seed.get("edges")
    paths = seed.get("learning_paths", [])
    if not isinstance(metadata, dict):
        errors.append(f"{seed_path.relative_to(ROOT)}: metadata must be an object")
        metadata = {}
    if not isinstance(nodes, list):
        errors.append(f"{seed_path.relative_to(ROOT)}: nodes must be an array")
        nodes = []
    if not isinstance(edges, list):
        errors.append(f"{seed_path.relative_to(ROOT)}: edges must be an array")
        edges = []
    if not isinstance(paths, list):
        errors.append(f"{seed_path.relative_to(ROOT)}: learning_paths must be an array")
        paths = []

    if (
        isinstance(metadata.get("total_standards"), bool)
        or metadata.get("total_standards") != len(nodes)
    ):
        errors.append(
            f"{seed_path.relative_to(ROOT)}: metadata.total_standards "
            f"must equal node count ({len(nodes)})"
        )

    allowed_node_types = {"Concept", "Skill", "Topic", "Course", "Assessment", "Project"}
    allowed_difficulties = {"Beginner", "Intermediate", "Advanced", "Expert"}
    allowed_bloom = {"Remember", "Understand", "Apply", "Analyze", "Evaluate", "Create"}
    allowed_subjects = {
        "Mathematics", "EnglishLanguageArts", "Science", "SocialStudies",
        "ForeignLanguage", "Arts", "PhysicalEducation", "Technology",
    }
    allowed_edge_types = {
        "Requires", "Recommends", "RelatedTo", "PartOf", "LeadsTo",
        "AlternativeTo", "Specializes", "AppliedIn",
    }

    ids: dict[str, dict[str, Any]] = {}
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            errors.append(f"{seed_path.relative_to(ROOT)}: node {index} must be an object")
            continue
        node_id = node.get("id")
        if not isinstance(node_id, str) or not node_id.strip():
            errors.append(f"{seed_path.relative_to(ROOT)}: node {index} has no stable id")
            continue
        if node_id in ids:
            errors.append(f"{seed_path.relative_to(ROOT)}: duplicate node id {node_id!r}")
            continue
        ids[node_id] = node

        if not isinstance(node.get("title"), str) or not node["title"].strip():
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} has no title")
        if not isinstance(node.get("description"), str) or "Demonstration:" not in node["description"]:
            errors.append(
                f"{seed_path.relative_to(ROOT)}: {node_id} must define an observable Demonstration"
            )
        if node.get("node_type") not in allowed_node_types:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} has invalid node_type")
        if node.get("difficulty") not in allowed_difficulties:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} has invalid difficulty")
        if node.get("bloom_level") not in allowed_bloom:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} has invalid bloom_level")
        if node.get("subject_area") not in allowed_subjects:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} has invalid subject_area")
        if not isinstance(node.get("grade_levels"), list) or not node["grade_levels"]:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} needs grade_levels")
        if not isinstance(node.get("academic_standards"), list):
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} academic_standards must be an array")
        if not isinstance(node.get("estimated_hours"), int) or node["estimated_hours"] <= 0:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} estimated_hours must be positive")
        if not isinstance(node.get("tags"), list) or "essential-capability-v1" not in node["tags"]:
            errors.append(f"{seed_path.relative_to(ROOT)}: {node_id} must carry the seed tag")

    adjacency: dict[str, list[str]] = {node_id: [] for node_id in ids}
    for index, edge in enumerate(edges):
        if not isinstance(edge, dict):
            errors.append(f"{seed_path.relative_to(ROOT)}: edge {index} must be an object")
            continue
        source = edge.get("from")
        target = edge.get("to")
        source_is_id = isinstance(source, str) and source in ids
        target_is_id = isinstance(target, str) and target in ids
        if not source_is_id or not target_is_id:
            errors.append(
                f"{seed_path.relative_to(ROOT)}: edge {index} references unknown nodes "
                f"{source!r} -> {target!r}"
            )
        if isinstance(source, str) and isinstance(target, str) and source == target:
            errors.append(f"{seed_path.relative_to(ROOT)}: edge {index} is a self-edge")
        if edge.get("edge_type") not in allowed_edge_types:
            errors.append(f"{seed_path.relative_to(ROOT)}: edge {index} has invalid edge_type")
        strength = edge.get("strength_permille")
        if isinstance(strength, bool) or not isinstance(strength, int) or not 0 <= strength <= 1000:
            errors.append(f"{seed_path.relative_to(ROOT)}: edge {index} has invalid strength_permille")
        if not isinstance(edge.get("rationale"), str) or not edge["rationale"].strip():
            errors.append(f"{seed_path.relative_to(ROOT)}: edge {index} needs a rationale")
        if (
            edge.get("edge_type") == "Requires"
            and source_is_id
            and target_is_id
        ):
            adjacency[source].append(target)

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node_id: str) -> None:
        if node_id in visiting:
            errors.append(
                f"{seed_path.relative_to(ROOT)}: cycle in strict Requires edges at {node_id!r}"
            )
            return
        if node_id in visited:
            return
        visiting.add(node_id)
        for target in adjacency.get(node_id, []):
            visit(target)
        visiting.remove(node_id)
        visited.add(node_id)

    for node_id in ids:
        visit(node_id)

    for index, path in enumerate(paths):
        if not isinstance(path, dict):
            errors.append(f"{seed_path.relative_to(ROOT)}: learning path {index} must be an object")
            continue
        sequence = path.get("sequence")
        if not isinstance(sequence, list) or not sequence:
            errors.append(f"{seed_path.relative_to(ROOT)}: learning path {index} needs a nonempty sequence")
            continue
        missing = [
            node_id for node_id in sequence
            if not isinstance(node_id, str) or node_id not in ids
        ]
        if missing:
            errors.append(
                f"{seed_path.relative_to(ROOT)}: learning path {index} has unknown IDs {missing!r}"
            )
            continue
        expected_hours = sum(ids[node_id]["estimated_hours"] for node_id in sequence)
        if path.get("total_hours") != expected_hours:
            errors.append(
                f"{seed_path.relative_to(ROOT)}: learning path {path.get('title')!r} "
                f"total_hours must be {expected_hours}, got {path.get('total_hours')!r}"
            )
        for prior, later in zip(sequence, sequence[1:]):
            if later not in adjacency.get(prior, []):
                # Suggested paths may intentionally cross a Recommends edge or omit
                # a strict edge, so this is informational rather than a validation error.
                pass

    actual_lesson_ids: set[str] = set()
    if not lesson_root.is_dir():
        errors.append(f"{lesson_root.relative_to(ROOT)}: essential-capability lesson directory missing")
    else:
        for lesson_path in sorted(lesson_root.glob("*.json")):
            lesson_doc = load_json(lesson_path, errors)
            if lesson_doc is None:
                continue
            node_id = lesson_doc.get("node_id")
            if not isinstance(node_id, str) or node_id not in ids:
                errors.append(
                    f"{lesson_path.relative_to(ROOT)}: lesson node_id is not in the seed: {node_id!r}"
                )
                continue
            if node_id in actual_lesson_ids:
                errors.append(f"{lesson_path.relative_to(ROOT)}: duplicate lesson for {node_id}")
                continue
            actual_lesson_ids.add(node_id)

            lesson = lesson_doc.get("lesson")
            if not isinstance(lesson, dict):
                errors.append(f"{lesson_path.relative_to(ROOT)}: lesson must be an object")
                continue
            if lesson.get("standard_code") != node_id:
                errors.append(f"{lesson_path.relative_to(ROOT)}: standard_code must equal node_id")
            for field in ("title", "explanation"):
                if not isinstance(lesson.get(field), str) or not lesson[field].strip():
                    errors.append(f"{lesson_path.relative_to(ROOT)}: lesson {field} is required")
            if not isinstance(lesson.get("examples"), list) or not lesson["examples"]:
                errors.append(f"{lesson_path.relative_to(ROOT)}: at least one worked example is required")
            practice = lesson.get("practice_problems")
            if not isinstance(practice, list) or len(practice) < 2:
                errors.append(f"{lesson_path.relative_to(ROOT)}: at least two practice problems are required")
            elif any(
                not isinstance(item, dict)
                or not all(isinstance(item.get(field), str) and item[field].strip()
                           for field in ("question", "answer", "explanation"))
                for item in practice
            ):
                errors.append(f"{lesson_path.relative_to(ROOT)}: each practice problem needs question, answer, and explanation")
            if not isinstance(lesson_doc.get("flashcards"), list) or not lesson_doc["flashcards"]:
                errors.append(f"{lesson_path.relative_to(ROOT)}: at least one flashcard is required")

            if manifest is not None:
                manifest_lessons = manifest.get("lessons", {})
                if not isinstance(manifest_lessons, dict):
                    errors.append("apps/leptos/static/content-manifest.json: lessons must be an object")
                    manifest_lessons = {}
                registered = manifest_lessons.get(node_id, [])
                if not isinstance(registered, list):
                    errors.append(
                        f"apps/leptos/static/content-manifest.json: {node_id} lesson entries must be an array"
                    )
                    registered = []
                expected_asset = f"/curriculum/generated/essential-capabilities/{lesson_path.name}"
                matching = [
                    item for item in registered
                    if isinstance(item, dict) and item.get("path") == expected_asset
                ]
                if len(matching) != 1:
                    errors.append(
                        f"{lesson_path.relative_to(ROOT)}: expected exactly one manifest entry "
                        f"for {expected_asset}"
                    )
                elif matching[0].get("title") != lesson.get("title"):
                    errors.append(
                        f"{lesson_path.relative_to(ROOT)}: manifest title differs from lesson title"
                    )

    missing_lessons = sorted(set(ids) - actual_lesson_ids)
    if missing_lessons:
        errors.append(
            f"{lesson_root.relative_to(ROOT)}: missing lessons for {', '.join(missing_lessons)}"
        )
    extra_lessons = sorted(actual_lesson_ids - set(ids))
    if extra_lessons:
        errors.append(
            f"{lesson_root.relative_to(ROOT)}: unexpected lesson IDs {', '.join(extra_lessons)}"
        )

    return len(ids), len(edges)


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
    essential_nodes, essential_edges = validate_essential_capabilities(errors)
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
        f"{active_lesson_nodes} active graph nodes with generated lessons, "
        f"{essential_nodes} essential capability nodes and {essential_edges} capability edges."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
