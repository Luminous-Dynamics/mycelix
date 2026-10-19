#!/usr/bin/env python3
"""Policy-neutral license declaration extractor for the Mycelix repository.

This tool records what repository surfaces say. It does not choose legal
precedence, infer relicensing authority, or decide intended policy.
"""

from __future__ import annotations

import argparse
import configparser
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError as exc:  # pragma: no cover - Python <3.11
    raise SystemExit("Python 3.11+ is required (tomllib unavailable)") from exc

PROFILE = "myc-ip-002a-license-declaration-extractor-v1"
LICENSE_FILENAMES = ("LICENSE", "LICENSE.md", "LICENSE.txt", "COPYING", "COPYING.md")
README_FILENAMES = ("README.md", "README.markdown")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="strict")


def detect_license_text(text: str) -> str:
    upper = text.upper()
    if "GNU AFFERO GENERAL PUBLIC LICENSE" in upper and re.search(r"\bVERSION\s+3\b", upper):
        return "AGPL-3.0-TEXT"
    if "APACHE LICENSE" in upper and re.search(r"\bVERSION\s+2\.0\b", upper):
        return "APACHE-2.0-TEXT"
    if "GNU GENERAL PUBLIC LICENSE" in upper and re.search(r"\bVERSION\s+3\b", upper):
        return "GPL-3.0-TEXT"
    if "MIT LICENSE" in upper or "PERMISSION IS HEREBY GRANTED, FREE OF CHARGE" in upper:
        return "MIT-TEXT"
    return "UNCLASSIFIED-TEXT"


def extract_readme_license_section(text: str) -> dict[str, Any] | None:
    lines = text.splitlines()
    heading_re = re.compile(r"^(#{1,6})\s+(.+?)\s*$")
    for index, line in enumerate(lines):
        match = heading_re.match(line)
        if not match or match.group(2).strip().casefold() not in {"license", "licensing"}:
            continue
        level = len(match.group(1))
        section: list[str] = []
        for later in lines[index + 1 :]:
            next_heading = heading_re.match(later)
            if next_heading and len(next_heading.group(1)) <= level:
                break
            section.append(later)
        while section and not section[0].strip():
            section.pop(0)
        while section and not section[-1].strip():
            section.pop()
        body = "\n".join(section).strip()
        return {
            "heading": match.group(2).strip(),
            "body": body,
            "first_nonempty_line": next((ln.strip() for ln in section if ln.strip()), ""),
        }
    return None


def cargo_license_declarations(path: Path) -> list[dict[str, str]]:
    with path.open("rb") as handle:
        parsed = tomllib.load(handle)
    found: list[dict[str, str]] = []
    package = parsed.get("package")
    if isinstance(package, dict) and isinstance(package.get("license"), str):
        found.append({"scope": "package", "value": package["license"]})
    workspace = parsed.get("workspace")
    if isinstance(workspace, dict):
        workspace_package = workspace.get("package")
        if isinstance(workspace_package, dict) and isinstance(workspace_package.get("license"), str):
            found.append({"scope": "workspace.package", "value": workspace_package["license"]})
    return found


def parse_submodule_paths(repo_root: Path) -> dict[str, str]:
    path = repo_root / ".gitmodules"
    if not path.is_file():
        return {}
    parser = configparser.ConfigParser()
    parser.read_string(read_text(path))
    result: dict[str, str] = {}
    for section in parser.sections():
        if not section.startswith("submodule "):
            continue
        sub_path = parser.get(section, "path", fallback="").strip()
        url = parser.get(section, "url", fallback="").strip()
        if sub_path:
            result[sub_path] = url
    return result


def discover_component_roots(repo_root: Path) -> list[Path]:
    roots = [repo_root]
    for entry in sorted(repo_root.iterdir(), key=lambda p: p.name):
        if entry.is_dir() and entry.name.startswith("mycelix-"):
            roots.append(entry)
    crates = repo_root / "crates"
    if crates.is_dir():
        for entry in sorted(crates.iterdir(), key=lambda p: p.name):
            if entry.is_dir():
                roots.append(entry)
    # Preserve order while avoiding duplicates.
    unique: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        resolved = root.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(root)
    return unique


def relative_posix(path: Path, root: Path) -> str:
    rel = path.relative_to(root)
    return "." if not rel.parts else rel.as_posix()


def file_record(path: Path, repo_root: Path) -> dict[str, Any]:
    return {
        "path": relative_posix(path, repo_root),
        "sha256": sha256_file(path),
        "bytes": path.stat().st_size,
    }


def scan_component(component_root: Path, repo_root: Path, submodules: dict[str, str]) -> dict[str, Any]:
    component_rel = relative_posix(component_root, repo_root)
    record: dict[str, Any] = {
        "component": component_rel,
        "repository_boundary": "submodule" if component_rel in submodules else "same_repository",
        "submodule_url": submodules.get(component_rel),
        "license_files": [],
        "cargo_manifests": [],
        "readmes": [],
    }

    for name in LICENSE_FILENAMES:
        candidate = component_root / name
        if candidate.is_file():
            text = read_text(candidate)
            item = file_record(candidate, repo_root)
            item["text_profile"] = detect_license_text(text)
            record["license_files"].append(item)

    cargo = component_root / "Cargo.toml"
    if cargo.is_file():
        item = file_record(cargo, repo_root)
        try:
            item["declarations"] = cargo_license_declarations(cargo)
            item["parse_status"] = "ok"
        except (tomllib.TOMLDecodeError, OSError, UnicodeError) as exc:
            item["declarations"] = []
            item["parse_status"] = "error"
            item["error"] = str(exc)
        record["cargo_manifests"].append(item)

    for name in README_FILENAMES:
        readme = component_root / name
        if readme.is_file():
            item = file_record(readme, repo_root)
            try:
                item["license_section"] = extract_readme_license_section(read_text(readme))
                item["parse_status"] = "ok"
            except (OSError, UnicodeError) as exc:
                item["license_section"] = None
                item["parse_status"] = "error"
                item["error"] = str(exc)
            record["readmes"].append(item)

    return record


def git_head(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    value = result.stdout.strip()
    return value or None


def build_inventory(repo_root: Path) -> dict[str, Any]:
    repo_root = repo_root.resolve()
    if not repo_root.is_dir():
        raise ValueError(f"repository root is not a directory: {repo_root}")
    submodules = parse_submodule_paths(repo_root)
    components = [scan_component(root, repo_root, submodules) for root in discover_component_roots(repo_root)]
    return {
        "extractor_profile": PROFILE,
        "git_head": git_head(repo_root),
        "repository_root_name": repo_root.name,
        "submodules": [
            {"path": path, "url": submodules[path]}
            for path in sorted(submodules)
        ],
        "components": components,
        "nonclaims": [
            "This inventory records repository declarations and does not determine legal precedence.",
            "This inventory does not establish copyright ownership or authority to relicense code.",
            "Absence of a declaration in an audited surface does not prove that no license applies.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo_root", nargs="?", type=Path, default=Path.cwd())
    parser.add_argument("--pretty", action="store_true", help="pretty-print JSON")
    args = parser.parse_args(argv)
    try:
        inventory = build_inventory(args.repo_root)
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if args.pretty:
        print(json.dumps(inventory, indent=2, sort_keys=True))
    else:
        print(json.dumps(inventory, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
