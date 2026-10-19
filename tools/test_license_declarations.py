#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import license_declarations as ld


class LicenseDeclarationExtractorTests(unittest.TestCase):
    def write(self, root: Path, rel: str, text: str) -> Path:
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        return path

    def test_readme_license_section_stops_at_peer_heading(self) -> None:
        section = ld.extract_readme_license_section(
            "# Demo\n\n## License\n\nAGPL-3.0-or-later\n\n### Note\nMore\n\n## Build\nNope\n"
        )
        self.assertIsNotNone(section)
        self.assertEqual(section["first_nonempty_line"], "AGPL-3.0-or-later")
        self.assertIn("More", section["body"])
        self.assertNotIn("Nope", section["body"])

    def test_detect_known_license_texts(self) -> None:
        self.assertEqual(ld.detect_license_text("GNU AFFERO GENERAL PUBLIC LICENSE\nVersion 3"), "AGPL-3.0-TEXT")
        self.assertEqual(ld.detect_license_text("Apache License\nVersion 2.0"), "APACHE-2.0-TEXT")
        self.assertEqual(ld.detect_license_text("unknown custom terms"), "UNCLASSIFIED-TEXT")

    def test_inventory_records_conflicting_surfaces_without_resolving_them(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "mycelix"
            root.mkdir()
            self.write(root, "LICENSE", "Apache License\nVersion 2.0\n")
            self.write(root, "README.md", "# Root\n\n## License\nApache-2.0\n")
            self.write(root, "mycelix-demo/LICENSE", "GNU AFFERO GENERAL PUBLIC LICENSE\nVersion 3\n")
            self.write(root, "mycelix-demo/Cargo.toml", '[workspace]\n[workspace.package]\nlicense = "AGPL-3.0-or-later"\n')
            self.write(root, "mycelix-demo/README.md", "# Demo\n\n## License\nApache-2.0\n")
            inventory = ld.build_inventory(root)
            demo = next(c for c in inventory["components"] if c["component"] == "mycelix-demo")
            self.assertEqual(demo["license_files"][0]["text_profile"], "AGPL-3.0-TEXT")
            self.assertEqual(demo["cargo_manifests"][0]["declarations"], [{"scope": "workspace.package", "value": "AGPL-3.0-or-later"}])
            self.assertEqual(demo["readmes"][0]["license_section"]["first_nonempty_line"], "Apache-2.0")
            self.assertNotIn("winner", json.dumps(demo).lower())

    def test_submodule_boundary_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "mycelix"
            root.mkdir()
            self.write(root, ".gitmodules", '[submodule "mycelix-health"]\n\tpath = mycelix-health\n\turl = https://example.invalid/health.git\n')
            (root / "mycelix-health").mkdir()
            inventory = ld.build_inventory(root)
            health = next(c for c in inventory["components"] if c["component"] == "mycelix-health")
            self.assertEqual(health["repository_boundary"], "submodule")
            self.assertEqual(health["submodule_url"], "https://example.invalid/health.git")

    def test_crate_roots_are_discovered(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "mycelix"
            (root / "crates" / "shared").mkdir(parents=True)
            self.write(root, "crates/shared/Cargo.toml", '[package]\nname = "shared"\nversion = "0.1.0"\nlicense = "MIT"\n')
            inventory = ld.build_inventory(root)
            shared = next(c for c in inventory["components"] if c["component"] == "crates/shared")
            self.assertEqual(shared["cargo_manifests"][0]["declarations"], [{"scope": "package", "value": "MIT"}])


if __name__ == "__main__":
    unittest.main()
