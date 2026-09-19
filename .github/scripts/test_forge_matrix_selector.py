#!/usr/bin/env python3
from __future__ import annotations

import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
SELECTOR_PATH = HERE / "forge_matrix_selector.py"
PROFILE_PATH = HERE.parent / "ci" / "forge_matrix_profile_v1.json"

spec = importlib.util.spec_from_file_location("forge_matrix_selector", SELECTOR_PATH)
selector = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(selector)


class ForgeMatrixSelectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = selector.load_profile(PROFILE_PATH)

    def ids(self, result):
        return [entry["id"] for entry in result["selected"]]

    def unbound(self, paths):
        return selector._select_unbound(paths, self.profile)

    def test_core_change_selects_full_current_graph(self):
        result = self.unbound(["crates/mycelix-forge-core/src/lib.rs"])
        self.assertEqual(result["disposition"], "selected_matrix")
        self.assertEqual(len(result["selected"]), 22)
        self.assertEqual(self.ids(result), [lane["id"] for lane in self.profile["lanes"]])

    def test_authority_change_is_leaf(self):
        result = self.unbound(["crates/mycelix-forge-authority/src/lib.rs"])
        self.assertEqual(self.ids(result), ["authority"])

    def test_repository_reverse_closure(self):
        result = self.unbound(["crates/mycelix-forge-repository/src/lib.rs"])
        self.assertEqual(
            self.ids(result),
            [
                "repository", "gittuf", "bundle", "policy-inventory", "guest-plan",
                "guest-transcript", "guest-tool-map", "guest-envelope", "guest-output-frame",
                "hermetic-guest", "hermetic-host",
            ],
        )

    def test_execution_reverse_closure(self):
        result = self.unbound(["crates/mycelix-forge-execution/src/lib.rs"])
        self.assertEqual(
            self.ids(result),
            ["execution", "guest-tool-map", "guest-envelope", "guest-output-frame", "hermetic-guest", "hermetic-host"],
        )

    def test_linux_isolation_reverse_closure(self):
        result = self.unbound(["adapters/forge-linux-isolation/src/lib.rs"])
        self.assertEqual(
            self.ids(result),
            [
                "linux-isolation", "isolation-evidence", "isolation-collector", "runtime-closure",
                "nar-auditor", "guest-tool-map", "guest-envelope", "guest-output-frame",
                "hermetic-guest", "sealed-inputs", "hermetic-host",
            ],
        )

    def test_guest_envelope_reverse_closure(self):
        result = self.unbound(["adapters/forge-guest-envelope/src/lib.rs"])
        self.assertEqual(self.ids(result), ["guest-envelope", "guest-output-frame", "hermetic-guest", "hermetic-host"])

    def test_semantic_guest_to_host_edge(self):
        result = self.unbound(["adapters/forge-hermetic-guest/src/main.rs"])
        self.assertEqual(self.ids(result), ["hermetic-guest", "hermetic-host"])

    def test_host_change_is_leaf(self):
        result = self.unbound(["adapters/forge-hermetic-host/src/lib.rs"])
        self.assertEqual(self.ids(result), ["hermetic-host"])

    def test_any_forge_cargo_manifest_forces_full_matrix(self):
        result = self.unbound(["adapters/forge-hermetic-host/Cargo.toml"])
        self.assertEqual(result["disposition"], "full_matrix")
        self.assertEqual(result["reason"], "forge_dependency_manifest_changed")
        self.assertEqual(len(result["selected"]), 22)

    def test_workflow_change_forces_full_matrix(self):
        result = self.unbound([".github/workflows/forge.yml"])
        self.assertEqual(result["disposition"], "full_matrix")
        self.assertEqual(result["reason"], "selector_profile_or_workflow_changed")

    def test_unknown_forge_path_forces_full_matrix(self):
        result = self.unbound(["adapters/forge-new-thing/src/lib.rs"])
        self.assertEqual(result["disposition"], "full_matrix")
        self.assertEqual(result["reason"], "unknown_forge_path")

    def test_profile_change_forces_full_matrix(self):
        result = self.unbound([".github/ci/forge_matrix_profile_v1.json"])
        self.assertEqual(result["disposition"], "full_matrix")

    def test_order_independence(self):
        a = self.unbound(
            ["adapters/forge-sealed-inputs/src/lib.rs", "adapters/forge-guest-envelope/src/lib.rs"]
        )
        b = self.unbound(
            ["adapters/forge-guest-envelope/src/lib.rs", "adapters/forge-sealed-inputs/src/lib.rs"]
        )
        self.assertEqual(
            json.dumps(a, sort_keys=True, separators=(",", ":")),
            json.dumps(b, sort_keys=True, separators=(",", ":")),
        )

    def test_multi_lane_union(self):
        result = self.unbound(
            ["crates/mycelix-forge-authority/src/lib.rs", "adapters/forge-sealed-inputs/src/lib.rs"]
        )
        self.assertEqual(self.ids(result), ["authority", "sealed-inputs", "hermetic-host"])

    def test_unrelated_path_only_is_no_forge_change(self):
        result = self.unbound(["docs/design.md"])
        self.assertEqual(result["disposition"], "no_forge_change")
        self.assertEqual(result["selected"], [])

    def test_unrelated_path_does_not_widen_known_selection(self):
        result = self.unbound(
            ["docs/design.md", "crates/mycelix-forge-authority/src/lib.rs"]
        )
        self.assertEqual(self.ids(result), ["authority"])

    def test_noncanonical_path_fails_closed(self):
        result = self.unbound(["../adapters/forge-hermetic-host/src/lib.rs"])
        self.assertEqual(result["disposition"], "full_matrix")
        self.assertEqual(result["reason"], "non_canonical_changed_path")

    def test_unknown_dependency_rejects_profile(self):
        broken = copy.deepcopy(self.profile)
        broken["lanes"][-1]["cargo_dependencies"].append("does-not-exist")
        with self.assertRaises(selector.ProfileError):
            selector.validate_profile(broken)

    def test_dependency_cycle_rejects_profile(self):
        broken = copy.deepcopy(self.profile)
        broken["lanes"][0]["semantic_dependencies"].append("hermetic-host")
        with self.assertRaises(selector.ProfileError):
            selector.validate_profile(broken)

    def test_lane_order_is_profile_order(self):
        result = self.unbound(["crates/mycelix-forge-core/src/lib.rs"])
        self.assertEqual(self.ids(result), [lane["id"] for lane in self.profile["lanes"]])

    def test_git_blob_sha_empty_known_vector(self):
        self.assertEqual(
            selector.git_blob_sha(b""),
            "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391",
        )

    def test_unknown_git_object_format_rejects_profile(self):
        broken = copy.deepcopy(self.profile)
        broken["git_object_format"] = "sha256"
        with self.assertRaises(selector.ProfileError):
            selector.validate_profile(broken)

    def test_bad_manifest_blob_sha_rejects_profile(self):
        broken = copy.deepcopy(self.profile)
        broken["lanes"][0]["cargo_manifest_blob_sha"] = "not-a-git-object"
        with self.assertRaises(selector.ProfileError):
            selector.validate_profile(broken)

    def _synthetic_bound_profile(self, root: Path):
        workflow = root / ".github" / "workflows" / "forge.yml"
        lane_root = root / "crates" / "mycelix-forge-core"
        workflow.parent.mkdir(parents=True)
        lane_root.mkdir(parents=True)
        workflow.write_bytes(b"name: Mycelix Forge CI\n")
        manifest = lane_root / "Cargo.toml"
        manifest.write_bytes(b'[package]\nname = "mycelix-forge-core"\n')

        profile = {
            "schema": selector.PROFILE_SCHEMA,
            "profile_version": 1,
            "audited_frontier": "synthetic",
            "git_object_format": "sha1",
            "workflow_path": ".github/workflows/forge.yml",
            "workflow_blob_sha": selector.git_blob_sha(workflow.read_bytes()),
            "forge_prefixes": ["crates/mycelix-forge-", "adapters/forge-"],
            "full_matrix_paths": [
                ".github/workflows/forge.yml",
                ".github/ci/forge_matrix_profile_v1.json",
                ".github/scripts/forge_matrix_selector.py",
                ".github/scripts/test_forge_matrix_selector.py",
            ],
            "lanes": [{
                "id": "core",
                "name": "Forge Core",
                "path": "crates/mycelix-forge-core",
                "cargo_manifest_blob_sha": selector.git_blob_sha(manifest.read_bytes()),
                "cargo_dependencies": [],
                "semantic_dependencies": [],
            }],
        }
        selector.validate_profile(profile)
        return profile, workflow, manifest

    def test_exact_source_bindings_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile, _, _ = self._synthetic_bound_profile(root)
            self.assertEqual(selector.source_binding_errors(root, profile), [])

    def test_workflow_binding_drift_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile, workflow, _ = self._synthetic_bound_profile(root)
            workflow.write_bytes(b"name: changed\n")
            errors = selector.source_binding_errors(root, profile)
            self.assertEqual(len(errors), 1)
            self.assertIn("workflow:blob_mismatch", errors[0])

    def test_manifest_binding_drift_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile, _, manifest = self._synthetic_bound_profile(root)
            manifest.write_bytes(b'[package]\nname = "changed"\n')
            errors = selector.source_binding_errors(root, profile)
            self.assertEqual(len(errors), 1)
            self.assertIn("lane:core:cargo_manifest:blob_mismatch", errors[0])

    def test_bound_selection_falls_back_full_on_drift(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile, _, manifest = self._synthetic_bound_profile(root)
            manifest.write_bytes(b'[package]\nname = "changed"\n')
            result = selector.select_matrix(
                ["crates/mycelix-forge-core/src/lib.rs"],
                profile,
                root,
            )
            self.assertEqual(result["disposition"], "full_matrix")
            self.assertEqual(result["reason"], "frozen_source_binding_mismatch")
            self.assertEqual(len(result["binding_errors"]), 1)

    def test_bound_selection_uses_graph_when_bindings_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile, _, _ = self._synthetic_bound_profile(root)
            result = selector.select_matrix(
                ["crates/mycelix-forge-core/src/lib.rs"],
                profile,
                root,
            )
            self.assertEqual(result["disposition"], "selected_matrix")
            self.assertEqual(self.ids(result), ["core"])


if __name__ == "__main__":
    unittest.main()
