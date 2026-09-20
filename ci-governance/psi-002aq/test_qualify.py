#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import os
import pathlib
import subprocess
import tempfile
import types
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(q)

GOOD_BACKEND = {
    "backend": {
        "crate": "voprf",
        "crate_version": "0.5.0",
        "crates_io_checksum_sha256": q.VOPRF_CHECKSUM,
        "repository": "https://github.com/facebook/voprf",
        "tag": "v0.5.0",
        "commit": q.VOPRF_UPSTREAM,
        "default_features": False,
        "features": ["ristretto255-ciphersuite"],
        "ciphersuite": "ristretto255-SHA512",
        "specification": "RFC 9497",
    },
    "data_profile": {"synthetic_only": True},
    "authority_ceiling": {
        "generic_psi_security_established": False,
        "enumeration_resistance_established": False,
        "client_anonymity_established": False,
        "transport_privacy_established": False,
        "registry_authenticity_established": False,
        "registry_freshness_established": False,
        "key_lifecycle_qualified": False,
        "wire_format_qualified": False,
        "real_data_admitted": False,
        "production_admitted": False,
        "application_authority_granted": False,
    },
}
GOOD_CARGO = 'voprf = { version = "=0.5.0", default-features = false, features = ["ristretto255-ciphersuite"] }\n'
GOOD_SOURCE = '''
pub struct SyntheticIdentifier(String);
fn x(v:&str){ let _=v.starts_with("synthetic-contact-"); }
pub struct BlindedRequest;
pub struct PendingQuery;
pub fn prepare_query() {}
pub fn evaluate_blinded(request: &BlindedRequest) { }
fn y() {
 VoprfClient::<Ristretto255>::blind;
 server.blind_evaluate();
 client.finalize();
 tag_set_commitment();
 simulate_online_membership_guess();
}
const C:&str="UPSTREAM";
fn receipt() {
 raw_identifier_snapshot_hash_emitted: false;
 server_request_contains_only_blinded_elements: true;
 offline_enumeration_resistance_established: false;
 online_enumeration_abuse_resistance_established: false;
 real_data_admitted: false;
 production_admitted: false;
 application_authority_granted: false;
}
'''.replace("UPSTREAM", q.VOPRF_UPSTREAM) + "\n".join(
    "#[test]\nfn t%d(){}" % index for index in range(q.EXPECTED_TESTS)
)
GOOD_LOCK = f'''version = 4
[[package]]
name = "voprf"
version = "0.5.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "{q.VOPRF_CHECKSUM}"
'''


def git(repo: pathlib.Path, *args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=repo, text=True).strip()


def init_qualifier_repo() -> pathlib.Path:
    root = pathlib.Path(tempfile.mkdtemp(prefix="psi002aq-test-"))
    subprocess.check_call(["git", "init", "-q"], cwd=root)
    subprocess.check_call(["git", "config", "user.name", "test"], cwd=root)
    subprocess.check_call(["git", "config", "user.email", "test@example.invalid"], cwd=root)
    target = root / q.PREFIX
    target.mkdir(parents=True)
    for name in q.QUALIFIER_NAMES:
        (target / name).write_text(f"{name}\n", encoding="utf-8")
    subprocess.check_call(["git", "add", "."], cwd=root)
    subprocess.check_call(["git", "commit", "-qm", "fixture"], cwd=root)
    return root


class Tests(unittest.TestCase):
    def test_canonical(self):
        self.assertEqual(q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')

    def test_git_env_rejected(self):
        with self.assertRaises(q.QualificationError):
            q.reject_git_env_overrides({"GIT_DIR": "/tmp/evil"})

    def test_dynamic_git_config_env_rejected(self):
        with self.assertRaises(q.QualificationError):
            q.reject_git_env_overrides({"GIT_CONFIG_KEY_0": "core.worktree"})

    def test_legacy_git_config_parameters_rejected(self):
        with self.assertRaises(q.QualificationError):
            q.reject_git_env_overrides({"GIT_CONFIG_PARAMETERS": "'core.worktree=/tmp/evil'"})

    def test_clean_env_disables_global_and_system_git_config(self):
        env = q.clean_env()
        self.assertEqual(env["GIT_CONFIG_GLOBAL"], os.devnull)
        self.assertEqual(env["GIT_CONFIG_NOSYSTEM"], "1")
        self.assertEqual(env["GIT_NO_REPLACE_OBJECTS"], "1")

    def test_build_env_rejects_rustc_wrapper(self):
        with self.assertRaises(q.QualificationError):
            q.reject_build_env_overrides({"RUSTC_WRAPPER": "/tmp/wrapper"})

    def test_build_env_rejects_profile_override(self):
        with self.assertRaises(q.QualificationError):
            q.reject_build_env_overrides({"CARGO_PROFILE_DEV_OVERFLOW_CHECKS": "false"})

    def test_clean_env_removes_semantic_build_overrides(self):
        old = os.environ.get("RUSTFLAGS")
        os.environ["RUSTFLAGS"] = "--cfg evil"
        try:
            self.assertNotIn("RUSTFLAGS", q.clean_env())
        finally:
            if old is None:
                os.environ.pop("RUSTFLAGS", None)
            else:
                os.environ["RUSTFLAGS"] = old

    def test_expected_lock_binds_execution_and_toolchain(self):
        lock = q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        self.assertEqual(lock["toolchain"]["rustc_release"], "1.96.0")
        self.assertEqual(lock["toolchain"]["cargo_release"], "1.96.0")
        self.assertEqual(lock["toolchain"]["rustc_commit_hash"], q.RUSTC_COMMIT)
        self.assertEqual(lock["toolchain"]["cargo_commit_hash"], q.CARGO_COMMIT)
        self.assertEqual(lock["qualifier"]["execution"]["python_argv_prefix"], ["python3", "-I", "-S", "-B"])
        self.assertTrue(lock["postflight"]["qualifier_working_bytes_reverified"])

    def test_python_flags_accept_isolated_profile(self):
        flags = types.SimpleNamespace(
            isolated=1,
            ignore_environment=1,
            no_user_site=1,
            safe_path=True,
            no_site=1,
        )
        q.verify_python_flags(flags, True)

    def test_python_flags_reject_nonisolated_profile(self):
        flags = types.SimpleNamespace(
            isolated=0,
            ignore_environment=0,
            no_user_site=0,
            safe_path=False,
            no_site=0,
        )
        with self.assertRaises(q.QualificationError):
            q.verify_python_flags(flags, True)

    def test_execution_path_accepts_canonical_file(self):
        with tempfile.TemporaryDirectory() as td:
            repo = pathlib.Path(td)
            canonical = repo / q.PREFIX / "qualify.py"
            canonical.parent.mkdir(parents=True)
            canonical.write_text("x\n")
            self.assertEqual(q.verify_execution_path(repo, canonical), str(canonical.resolve()))

    def test_execution_path_rejects_copy(self):
        with tempfile.TemporaryDirectory() as td:
            repo = pathlib.Path(td)
            canonical = repo / q.PREFIX / "qualify.py"
            canonical.parent.mkdir(parents=True)
            canonical.write_text("x\n")
            copied = repo / "copy.py"
            copied.write_text("x\n")
            with self.assertRaises(q.QualificationError):
                q.verify_execution_path(repo, copied)

    def test_working_file_verification_accepts_exact_head_bytes(self):
        repo = init_qualifier_repo()
        try:
            evidence = q.verify_working_qualifier_files(repo)
            self.assertEqual(set(evidence), set(q.QUALIFIER_NAMES))
            self.assertEqual(evidence["qualify.py"]["blob"], git(repo, "rev-parse", f"HEAD:{q.PREFIX}/qualify.py"))
        finally:
            subprocess.call(["rm", "-rf", str(repo)])

    def test_working_file_verification_rejects_modified_bytes(self):
        repo = init_qualifier_repo()
        try:
            (repo / q.PREFIX / "README.md").write_text("mutated\n")
            with self.assertRaises(q.QualificationError):
                q.verify_working_qualifier_files(repo)
        finally:
            subprocess.call(["rm", "-rf", str(repo)])

    def test_dangerous_local_git_config_rejected(self):
        repo = init_qualifier_repo()
        try:
            subprocess.check_call(["git", "config", "--local", "include.path", "/tmp/evil"], cwd=repo)
            with self.assertRaises(q.QualificationError):
                q.reject_dangerous_local_config(repo)
        finally:
            subprocess.call(["rm", "-rf", str(repo)])

    def test_cargo_config_rejects_workspace_ancestor_config(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            workspace = root / "workspace"
            workspace.mkdir()
            cfg = root / ".cargo" / "config.toml"
            cfg.parent.mkdir()
            cfg.write_text("[build]\nrustflags=['--cfg','evil']\n")
            with self.assertRaises(q.QualificationError):
                q.reject_cargo_config(workspace, {"HOME": str(root / "home")})

    def test_cargo_config_accepts_clean_search_path(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            workspace = root / "workspace"
            workspace.mkdir()
            q.reject_cargo_config(workspace, {"CARGO_HOME": str(root / "cargo-home"), "HOME": str(root / "home")})

    def test_backend_lock_accepts(self):
        q.verify_backend_lock_bytes(json.dumps(GOOD_BACKEND).encode())

    def test_backend_lock_rejects_checksum(self):
        bad = json.loads(json.dumps(GOOD_BACKEND))
        bad["backend"]["crates_io_checksum_sha256"] = "0" * 64
        with self.assertRaises(q.QualificationError):
            q.verify_backend_lock_bytes(json.dumps(bad).encode())

    def test_cargo_lock_accepts_checksum(self):
        self.assertEqual(q.verify_voprf_cargo_lock(GOOD_LOCK.encode())["checksum"], q.VOPRF_CHECKSUM)

    def test_cargo_lock_rejects_checksum(self):
        with self.assertRaises(q.QualificationError):
            q.verify_voprf_cargo_lock(GOOD_LOCK.replace(q.VOPRF_CHECKSUM, "f" * 64).encode())

    def test_role_probe_accepts(self):
        self.assertEqual(
            q.static_source_probes(GOOD_SOURCE, GOOD_CARGO, json.dumps(GOOD_BACKEND).encode())["product_test_count"],
            q.EXPECTED_TESTS,
        )

    def test_role_probe_rejects_raw_server_parameter(self):
        bad = GOOD_SOURCE.replace(
            "pub fn evaluate_blinded(request: &BlindedRequest)",
            "pub fn evaluate_blinded(request: &BlindedRequest, raw: &SyntheticIdentifier)",
        )
        with self.assertRaises(q.QualificationError):
            q.static_source_probes(bad, GOOD_CARGO, json.dumps(GOOD_BACKEND).encode())

    def test_role_probe_rejects_raw_snapshot_hash(self):
        with self.assertRaises(q.QualificationError):
            q.static_source_probes(
                GOOD_SOURCE + "\nfn snapshot_commitment() {}\n",
                GOOD_CARGO,
                json.dumps(GOOD_BACKEND).encode(),
            )

    def test_role_probe_rejects_authority_promotion(self):
        bad = GOOD_SOURCE.replace("production_admitted: false", "production_admitted: true")
        with self.assertRaises(q.QualificationError):
            q.static_source_probes(bad, GOOD_CARGO, json.dumps(GOOD_BACKEND).encode())

    def test_role_probe_rejects_danger_feature(self):
        with self.assertRaises(q.QualificationError):
            q.static_source_probes(
                GOOD_SOURCE,
                GOOD_CARGO + 'features = ["danger"]\n',
                json.dumps(GOOD_BACKEND).encode(),
            )

    def test_test_count_exact(self):
        bad = GOOD_SOURCE.rsplit("#[test]", 1)[0]
        with self.assertRaises(q.QualificationError):
            q.static_source_probes(bad, GOOD_CARGO, json.dumps(GOOD_BACKEND).encode())

    def rustc_record(self, commit=None, release="1.96.0"):
        commit = q.RUSTC_COMMIT if commit is None else commit
        return {"argv": ["rustc", "-Vv"], "returncode": 0, "stdout": f"rustc {release} (x)\ncommit-hash: {commit}\ncommit-date: {q.RUSTC_COMMIT_DATE}\nrelease: {release}\nhost: x86_64-unknown-linux-gnu\n", "stderr": ""}

    def cargo_record(self, commit=None, release="1.96.0"):
        commit = q.CARGO_COMMIT if commit is None else commit
        return {"argv": ["cargo", "-Vv"], "returncode": 0, "stdout": f"cargo {release} (x)\nrelease: {release}\ncommit-hash: {commit}\ncommit-date: {q.CARGO_COMMIT_DATE}\nhost: x86_64-unknown-linux-gnu\n", "stderr": ""}

    def test_toolchain_accepts_exact_196_source_identities(self):
        observed = q.verify_toolchain_versions(self.rustc_record(), self.cargo_record())
        self.assertEqual(observed["rustc_commit_hash"], q.RUSTC_COMMIT)
        self.assertEqual(observed["cargo_commit_hash"], q.CARGO_COMMIT)

    def test_toolchain_rejects_wrong_rustc_release(self):
        with self.assertRaises(q.QualificationError):
            q.verify_toolchain_versions(self.rustc_record(release="1.97.0"), self.cargo_record())

    def test_toolchain_rejects_wrong_rustc_commit(self):
        with self.assertRaises(q.QualificationError):
            q.verify_toolchain_versions(self.rustc_record(commit="0" * 40), self.cargo_record())

    def test_toolchain_rejects_wrong_cargo_commit(self):
        with self.assertRaises(q.QualificationError):
            q.verify_toolchain_versions(self.rustc_record(), self.cargo_record(commit="f" * 40))


if __name__ == "__main__":
    unittest.main()
