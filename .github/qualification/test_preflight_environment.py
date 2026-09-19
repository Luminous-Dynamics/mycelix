#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "envmod", Path(__file__).with_name("preflight_environment.py")
)
assert SPEC and SPEC.loader
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


class T(unittest.TestCase):
    def executable(
        self, root: Path, name: str, body: str = "exit 0\n"
    ) -> Path:
        path = root / name
        path.write_text("#!/bin/sh\n" + body)
        path.chmod(0o755)
        return path

    def test_identity_detects_symlink_retarget_and_byte_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            a = self.executable(root, "a")
            b = self.executable(root, "b")
            link = root / "tool"
            link.symlink_to(a)
            identity = m.identity_for_path("tool", link)
            self.assertTrue(m.identity_still_matches(identity))
            link.unlink()
            link.symlink_to(b)
            self.assertFalse(m.identity_still_matches(identity))
            link.unlink()
            link.symlink_to(a)
            identity = m.identity_for_path("tool", link)
            a.write_text("#!/bin/sh\nexit 7\n")
            a.chmod(0o755)
            self.assertFalse(m.identity_still_matches(identity))

    def test_profile_binding_rejects_duplicate_invalid_and_oversize(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "p.json"
            path.write_text('{"toolchain":"1.98.1"}')
            binding = m._bind_profile(path)
            self.assertEqual(binding["toolchain"], "1.98.1")
            self.assertRegex(binding["profile_commitment"], r"^[0-9a-f]{64}$")
            self.assertRegex(binding["raw_sha256"], r"^[0-9a-f]{64}$")

            path.write_text(
                '{"toolchain":"1.98.1","toolchain":"1.98.0"}'
            )
            with self.assertRaises(m.EnvironmentError):
                m._bind_profile(path)

            path.write_text('{"toolchain":"nightly"}')
            with self.assertRaises(m.EnvironmentError):
                m._bind_profile(path)

            path.write_bytes(b" " * (m.MAX_PROFILE + 1))
            with self.assertRaises(m.EnvironmentError):
                m._bind_profile(path)

    def test_profile_commitment_is_canonical_but_raw_identity_is_not(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            a = root / "a.json"
            b = root / "b.json"
            a.write_text('{"toolchain":"1.98.1","x":1}')
            b.write_text('{"x":1,"toolchain":"1.98.1"}')
            aa = m._bind_profile(a)
            bb = m._bind_profile(b)
            self.assertEqual(aa["profile_commitment"], bb["profile_commitment"])
            self.assertNotEqual(aa["raw_sha256"], bb["raw_sha256"])

    def test_commitment_is_canonical(self):
        a = {"b": 2, "a": 1}
        b = {"a": 1, "b": 2}
        self.assertEqual(m._canonical_commitment(a), m._canonical_commitment(b))
        b["a"] = 3
        self.assertNotEqual(m._canonical_commitment(a), m._canonical_commitment(b))

    def test_child_env_is_minimal_and_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            env = m._child_env(root, bin_dir, "/tmp/rh")
            self.assertEqual(env["PATH"], str(bin_dir))
            self.assertEqual(env["RUSTUP_HOME"], "/tmp/rh")
            self.assertEqual(env["PYTHONHASHSEED"], "0")
            self.assertEqual(
                set(env),
                {
                    "PATH",
                    "HOME",
                    "TMPDIR",
                    "XDG_CONFIG_HOME",
                    "XDG_CACHE_HOME",
                    "RUSTUP_HOME",
                    "LC_ALL",
                    "LANG",
                    "TZ",
                    "PYTHONHASHSEED",
                },
            )

    def test_explicit_relative_rustup_home_is_rejected(self):
        old = os.environ.copy()
        try:
            os.environ["RUSTUP_HOME"] = "relative-rustup"
            with self.assertRaises(m.EnvironmentUnavailable):
                m._source_rustup_home()
        finally:
            os.environ.clear()
            os.environ.update(old)

    def test_bound_links_target_frozen_resolved_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = self.executable(root, "real")
            discovered = root / "found"
            discovered.symlink_to(target)
            identity = m.identity_for_path("tool", discovered)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            m._install_bound_name(bin_dir, "tool", identity)
            self.assertEqual((bin_dir / "tool").resolve(), target.resolve())

    def test_environment_manifest_separates_host_identity(self):
        def fake(name: str) -> dict:
            return {
                "requested_name": name,
                "discovered_path": "/x",
                "symlink_chain": ["/x"],
                "resolved_path": "/x",
                "discovered_mode": 493,
                "resolved_mode": 493,
                "size": 1,
                "device": 1,
                "inode": 1,
                "sha256": "0" * 64,
            }

        manifest = m._environment_manifest(
            fake("python"),
            fake("git"),
            fake("rustup"),
            "/rh",
            "1.98.1",
            fake("cargo"),
            fake("rustfmt"),
            fake("cargo-fmt"),
        )
        self.assertEqual(manifest["toolchain"], "1.98.1")
        self.assertEqual(manifest["policy_revision"], m.POLICY_REVISION)
        self.assertTrue(manifest["python_isolated_mode"])
        self.assertEqual(manifest["rustup_home"], "/rh")
        self.assertRegex(m._canonical_commitment(manifest), r"^[0-9a-f]{64}$")

    def _integration_fixture(
        self,
        root: Path,
        *,
        child_overrides: dict | None = None,
        exit_code: int = 0,
        mutate_profile: bool = False,
        mutate_child_source: bool = False,
        resolver_selector: Path | None = None,
        resolver_target: Path | None = None,
    ):
        host_bin = root / "hostbin"
        host_bin.mkdir()
        old = os.environ.copy()
        git_real = shutil.which("git")
        assert git_real
        (host_bin / "git").symlink_to(git_real)

        toolroot = root / "toolchain"
        toolroot.mkdir()
        for name in ("cargo", "rustfmt", "cargo-fmt"):
            self.executable(toolroot, name)

        rustup = host_bin / "rustup"
        if resolver_selector is None:
            rustup.write_text(
                "#!/bin/sh\n"
                'if [ "$1" = which ] && [ "$2" = --toolchain ] '
                '&& [ "$3" = 1.98.1 ]; then '
                f'printf "%s/%s\\n" {str(toolroot)!r} "$4"; exit 0; fi\n'
                "exit 64\n"
            )
        else:
            rustup.write_text(
                "#!/bin/sh\n"
                'if [ "$1" = which ]; then '
                f'IFS= read -r root < {str(resolver_selector)!r}; '
                'printf "%s/%s\\n" "$root" "$4"; exit 0; fi\n'
                "exit 64\n"
            )
        rustup.chmod(0o755)

        profile = root / "profile.json"
        profile.write_text(json.dumps({"toolchain": "1.98.1", "x": 1}))
        expected_profile_commitment = m._bind_profile(profile)["profile_commitment"]

        child = root / "preflight.py"
        result = {
            "schema": m.CHILD_SCHEMA,
            "classification": "ELIGIBLE",
            "qualification_result": None,
            "qualification_authority": False,
            "profile_commitment": expected_profile_commitment,
        }
        if child_overrides:
            result.update(child_overrides)

        statements: list[str] = []
        if mutate_profile:
            statements.append(
                f"open({str(profile)!r},'w').write(" 
                + repr(json.dumps({"toolchain": "1.98.0", "x": 2}))
                + ")"
            )
        if resolver_selector is not None and resolver_target is not None:
            statements.append(
                f"open({str(resolver_selector)!r},'w').write({str(resolver_target)!r})"
            )

        template = '''#!/usr/bin/env python3
import hashlib,json,sys
D=b"MYCELIX_QUALIFICATION_PREFLIGHT_IMPLEMENTATION_V1\\0"
B=open(__file__,"rb").read()
R=__RESULT__
R["preflight_implementation_commitment"]=hashlib.sha256(D+B).hexdigest()
__STATEMENTS__
print(json.dumps(R,separators=(",",":")))
raise SystemExit(__EXIT__)
'''
        child.write_text(
            template.replace("__RESULT__", repr(result))
            .replace("__STATEMENTS__", "\n".join(statements))
            .replace("__EXIT__", str(exit_code))
        )

        # Source-child drift requires a second external actor because the bound
        # child executes a copy. Use the child itself to mutate the source path.
        if mutate_child_source:
            text = child.read_text()
            marker = "print(json.dumps(R,separators=(\",\",\":\")))"
            replacement = (
                f"open({str(child)!r},'a').write('\\n# mutated source\\n')\n"
                + marker
            )
            child.write_text(text.replace(marker, replacement))

        os.environ.clear()
        os.environ.update(old)
        os.environ["PATH"] = str(host_bin)
        os.environ["RUSTUP_HOME"] = str(root / "rustup-home")
        return old, profile, child

    def test_execute_bound_accepts_exact_child_contract_and_augments_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, profile, child = self._integration_fixture(root)
            try:
                out = m.execute_bound(
                    repo=root,
                    profile=profile,
                    subject="0" * 40,
                    preflight_script=child,
                )
            finally:
                os.environ.clear()
                os.environ.update(old)
            self.assertEqual(out["classification"], "ELIGIBLE")
            self.assertEqual(out["environment_adapter_schema"], m.SCHEMA)
            self.assertEqual(
                out["bound_profile_commitment"], out["profile_commitment"]
            )
            self.assertRegex(
                out["preflight_environment_commitment"], r"^[0-9a-f]{64}$"
            )
            self.assertEqual(out["child_process"]["exit_code"], 0)

    def test_execute_bound_rejects_schema_authority_exit_and_profile_mismatch(self):
        cases = [
            ({"schema": "wrong"}, 0),
            ({"qualification_result": "PASS"}, 0),
            ({"qualification_authority": True}, 0),
            ({}, 4),
            ({"profile_commitment": "0" * 64}, 0),
        ]
        for overrides, code in cases:
            with self.subTest(overrides=overrides, code=code):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    old, profile, child = self._integration_fixture(
                        root, child_overrides=overrides, exit_code=code
                    )
                    try:
                        with self.assertRaises(m.EnvironmentError):
                            m.execute_bound(
                                repo=root,
                                profile=profile,
                                subject="0" * 40,
                                preflight_script=child,
                            )
                    finally:
                        os.environ.clear()
                        os.environ.update(old)

    def test_execute_bound_rejects_post_run_rustup_resolver_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            a = root / "toolchain-a"
            b = root / "toolchain-b"
            a.mkdir()
            b.mkdir()
            for toolroot in (a, b):
                for name in ("cargo", "rustfmt", "cargo-fmt"):
                    self.executable(toolroot, name)
            selector = root / "selector"
            selector.write_text(str(a))
            old, profile, child = self._integration_fixture(
                root,
                resolver_selector=selector,
                resolver_target=b,
            )
            try:
                with self.assertRaises(m.EnvironmentError):
                    m.execute_bound(
                        repo=root,
                        profile=profile,
                        subject="0" * 40,
                        preflight_script=child,
                    )
            finally:
                os.environ.clear()
                os.environ.update(old)

    def test_execute_bound_rejects_profile_source_drift_after_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, profile, child = self._integration_fixture(
                root, mutate_profile=True
            )
            try:
                with self.assertRaisesRegex(
                    m.EnvironmentError, "profile source drift after launch"
                ):
                    m.execute_bound(
                        repo=root,
                        profile=profile,
                        subject="0" * 40,
                        preflight_script=child,
                    )
            finally:
                os.environ.clear()
                os.environ.update(old)

    def test_execute_bound_rejects_child_source_drift_after_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, profile, child = self._integration_fixture(
                root, mutate_child_source=True
            )
            try:
                with self.assertRaisesRegex(
                    m.EnvironmentError, "child source drift after launch"
                ):
                    m.execute_bound(
                        repo=root,
                        profile=profile,
                        subject="0" * 40,
                        preflight_script=child,
                    )
            finally:
                os.environ.clear()
                os.environ.update(old)

    def test_preflight_child_commitment_must_match_exact_bound_script_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, profile, child = self._integration_fixture(
                root,
                child_overrides={
                    "preflight_implementation_commitment": "0" * 64
                },
                exit_code=4,
            )
            # Override the self-computing child so it lies about its identity.
            expected_profile = m._bind_profile(profile)["profile_commitment"]
            result = {
                "schema": m.CHILD_SCHEMA,
                "classification": "INVALID",
                "qualification_result": None,
                "qualification_authority": False,
                "profile_commitment": expected_profile,
                "preflight_implementation_commitment": "0" * 64,
            }
            child.write_text(
                "#!/usr/bin/env python3\nimport json\n"
                + f"print(json.dumps({result!r}))\n"
                + "raise SystemExit(4)\n"
            )
            try:
                with self.assertRaises(m.EnvironmentError):
                    m.execute_bound(
                        repo=root,
                        profile=profile,
                        subject="0" * 40,
                        preflight_script=child,
                    )
            finally:
                os.environ.clear()
                os.environ.update(old)

    def test_bound_inputs_are_exact_copies_not_source_paths(self):
        # A valid child sees the adapter-owned profile copy. The resulting
        # profile commitment therefore equals the pre-bound canonical identity.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, profile, child = self._integration_fixture(root)
            original_profile = profile.resolve()
            try:
                out = m.execute_bound(
                    repo=root,
                    profile=profile,
                    subject="0" * 40,
                    preflight_script=child,
                )
            finally:
                os.environ.clear()
                os.environ.update(old)
            self.assertEqual(
                out["bound_profile_raw_sha256"],
                m._sha256_file(original_profile),
            )
            self.assertEqual(
                out["bound_profile_commitment"], out["profile_commitment"]
            )


if __name__ == "__main__":
    unittest.main()
