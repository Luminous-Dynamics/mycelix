import importlib.util
import io
import json
import os
import pathlib
import subprocess
import tarfile
import tempfile
import unittest
from unittest import mock

HERE = pathlib.Path(__file__).parent
SPEC = importlib.util.spec_from_file_location(
    "qualifier", str(HERE / "ci_gov_001d_exact_qualify.py")
)
qualifier = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(qualifier)


class ExactQualifierTests(unittest.TestCase):
    def test_canonical_hash_is_order_independent(self):
        a = {"b": 2, "a": [3, 1]}
        b = {"a": [3, 1], "b": 2}
        self.assertEqual(qualifier.canonical_sha256(a), qualifier.canonical_sha256(b))
        self.assertEqual(len(qualifier.canonical_sha256(a)), 64)

    def test_expected_lineage_is_frozen(self):
        self.assertEqual(
            qualifier.BASE_001A,
            "884a14e14758a91d3c1d370d49648946dc8b89ef",
        )
        self.assertEqual(
            qualifier.PLANNER,
            "4fc54eee4bc8a590d8cbe348bfa81ba649ddd1a0",
        )
        self.assertEqual(
            qualifier.EXECUTOR,
            "b55758c19bb60e3a9266f2144116dee10c5de274",
        )
        self.assertEqual(
            qualifier.SCHEMA,
            "mycelix.ci-gov.001d.exact-qualification.v0.2",
        )

    def test_expected_path_sets_are_exact(self):
        self.assertEqual(
            sorted(qualifier.PLANNER_BLOBS),
            [
                ".github/scripts/CI_GOV_001D_A.md",
                ".github/scripts/ci_superseded_run_plan.py",
                ".github/scripts/test_ci_superseded_run_plan.py",
            ],
        )
        self.assertEqual(
            sorted(qualifier.EXECUTOR_BLOBS),
            [
                ".github/scripts/CI_GOV_001D_B.lock.json",
                ".github/scripts/CI_GOV_001D_B.md",
                ".github/scripts/ci_superseded_run_execute.py",
                ".github/scripts/test_ci_superseded_run_execute.py",
            ],
        )
        self.assertEqual(
            sorted(qualifier.QUALIFIER_PATHS),
            [
                ".github/scripts/CI_GOV_001D_Q.md",
                ".github/scripts/ci_gov_001d_exact_qualify.py",
                ".github/scripts/test_ci_gov_001d_exact_qualify.py",
            ],
        )

    def test_subject_archive_scope_is_exact(self):
        self.assertEqual(
            qualifier.SUBJECT_ARCHIVE_PATHS,
            [
                ".github/scripts/ci_queue_census.py",
                ".github/scripts/ci_superseded_run_plan.py",
                ".github/scripts/test_ci_superseded_run_plan.py",
                ".github/scripts/ci_superseded_run_execute.py",
                ".github/scripts/test_ci_superseded_run_execute.py",
            ],
        )
        with mock.patch.object(__import__("sys"), "argv", ["qualifier"]):
            args = qualifier.parse_args()
        self.assertFalse(hasattr(args, "archive_path"))
        self.assertFalse(hasattr(args, "subject_path"))

    def test_archive_file_allowlist_rejects_extra(self):
        qualifier.verify_archive_files(list(qualifier.SUBJECT_ARCHIVE_PATHS))
        with self.assertRaises(qualifier.QualificationError):
            qualifier.verify_archive_files(
                list(qualifier.SUBJECT_ARCHIVE_PATHS) + ["unexpected.txt"]
            )

    def test_git_environment_disables_replace_objects(self):
        env = qualifier.git_env()
        self.assertEqual(env["GIT_NO_REPLACE_OBJECTS"], "1")

    def test_git_environment_defeats_real_replace_ref(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            subprocess.run(["git", "init", "-q"], cwd=root, check=True)

            good = subprocess.run(
                ["git", "hash-object", "-w", "--stdin"],
                cwd=root,
                input=b"good\n",
                stdout=subprocess.PIPE,
                check=True,
            ).stdout.decode().strip()
            evil = subprocess.run(
                ["git", "hash-object", "-w", "--stdin"],
                cwd=root,
                input=b"evil\n",
                stdout=subprocess.PIPE,
                check=True,
            ).stdout.decode().strip()
            subprocess.run(
                ["git", "update-ref", f"refs/replace/{good}", evil],
                cwd=root,
                check=True,
            )

            ordinary = subprocess.run(
                ["git", "cat-file", "-p", good],
                cwd=root,
                stdout=subprocess.PIPE,
                check=True,
            ).stdout
            self.assertEqual(ordinary, b"evil\n")
            self.assertEqual(qualifier.git_bytes("cat-file", "-p", good, cwd=root), b"good\n")

    def test_git_rewrite_state_rejects_redirect_environment(self):
        with mock.patch.dict(os.environ, {"GIT_DIR": "/tmp/not-the-checkout"}, clear=False):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.ensure_no_git_rewrite_state()

    def test_git_rewrite_state_rejects_replace_refs(self):
        with mock.patch.object(
            qualifier,
            "git_text",
            return_value="refs/replace/0123456789012345678901234567890123456789",
        ):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.ensure_no_git_rewrite_state()

    def test_git_rewrite_state_rejects_grafts(self):
        with tempfile.TemporaryDirectory() as tmp:
            grafts = pathlib.Path(tmp) / "grafts"
            grafts.write_text("deadbeef parent\n")

            def fake_git_text(*args, **kwargs):
                if args[:2] == ("for-each-ref", "--format=%(refname)"):
                    return ""
                if args == ("rev-parse", "--git-path", "info/grafts"):
                    return str(grafts)
                raise AssertionError((args, kwargs))

            with mock.patch.object(qualifier, "git_text", side_effect=fake_git_text):
                with self.assertRaises(qualifier.QualificationError):
                    qualifier.ensure_no_git_rewrite_state()

    def test_qualifier_checkout_requires_direct_parent_exact_paths_and_clean_tree(self):
        def fake_git_text(*args, **kwargs):
            if args == ("rev-parse", "HEAD"):
                return "qualifier-head"
            if args == ("rev-parse", "--show-toplevel"):
                return "/repo"
            if args == ("status", "--porcelain", "--untracked-files=all"):
                return ""
            raise AssertionError((args, kwargs))

        with mock.patch.object(qualifier, "git_text", side_effect=fake_git_text), \
             mock.patch.object(qualifier, "commit_parents", return_value=[qualifier.EXECUTOR]), \
             mock.patch.object(qualifier, "changed_paths", return_value=sorted(qualifier.QUALIFIER_PATHS)), \
             mock.patch.object(qualifier, "verify_qualifier_files_match_head"), \
             mock.patch.object(qualifier, "__file__", "/repo/.github/scripts/ci_gov_001d_exact_qualify.py"):
            head, root = qualifier.verify_qualifier_checkout()
        self.assertEqual(head, "qualifier-head")
        self.assertEqual(root, pathlib.Path("/repo"))

    def test_qualifier_working_files_match_head(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            for path in qualifier.QUALIFIER_PATHS:
                target = root / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(path.encode())

            def fake_show(_head, path):
                return path.encode()

            with mock.patch.object(qualifier, "git_show", side_effect=fake_show):
                qualifier.verify_qualifier_files_match_head(root, "head")

    def test_qualifier_working_file_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            for path in qualifier.QUALIFIER_PATHS:
                target = root / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(path.encode())
            with mock.patch.object(qualifier, "git_show", return_value=b"different"):
                with self.assertRaises(qualifier.QualificationError):
                    qualifier.verify_qualifier_files_match_head(root, "head")

    def test_qualifier_checkout_rejects_dirty_tree(self):
        def fake_git_text(*args, **kwargs):
            if args == ("rev-parse", "HEAD"):
                return "qualifier-head"
            if args == ("rev-parse", "--show-toplevel"):
                return "/repo"
            if args == ("status", "--porcelain", "--untracked-files=all"):
                return "?? receipt.json"
            raise AssertionError((args, kwargs))

        with mock.patch.object(qualifier, "git_text", side_effect=fake_git_text), \
             mock.patch.object(qualifier, "commit_parents", return_value=[qualifier.EXECUTOR]), \
             mock.patch.object(qualifier, "changed_paths", return_value=sorted(qualifier.QUALIFIER_PATHS)), \
             mock.patch.object(qualifier, "__file__", "/repo/.github/scripts/ci_gov_001d_exact_qualify.py"):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.verify_qualifier_checkout()

    def test_receipt_output_must_be_outside_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            checkout = root / "repo"
            checkout.mkdir()
            outside = root / "receipt.json"
            inside = checkout / "receipt.json"
            with mock.patch.object(qualifier, "git_text", return_value=str(checkout)):
                qualifier.ensure_receipt_output_outside_checkout(None)
                qualifier.ensure_receipt_output_outside_checkout(outside)
                with self.assertRaises(qualifier.QualificationError):
                    qualifier.ensure_receipt_output_outside_checkout(inside)
                with self.assertRaises(qualifier.QualificationError):
                    qualifier.ensure_receipt_output_outside_checkout(checkout)

    def test_lock_verification_accepts_only_exact_contract(self):
        payload = (json.dumps(qualifier.EXPECTED_LOCK, sort_keys=True) + "\n").encode()
        with mock.patch.object(qualifier, "git_show", return_value=payload):
            observed = qualifier.verify_lock()
        self.assertEqual(observed, qualifier.sha256_bytes(payload))

        changed = dict(qualifier.EXPECTED_LOCK)
        changed["mutation_contract"] = dict(changed["mutation_contract"])
        changed["mutation_contract"]["max_selected_runs"] = 6
        with mock.patch.object(
            qualifier, "git_show", return_value=json.dumps(changed).encode()
        ):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.verify_lock()

    def test_authority_surface_happy_path(self):
        executor = '''\nrequest = urllib.request.Request(\n    url, method="POST"\n)\npath = f"/repos/{repository}/actions/runs/{run_id}/cancel"\n'''
        planner = "from ci_queue_census import GitHubReadOnlyClient\n"
        readonly = 'request = urllib.request.Request(url, method="GET")\n'

        def fake_show(commit, path):
            if path.endswith("ci_superseded_run_execute.py"):
                return executor.encode()
            if path.endswith("ci_superseded_run_plan.py"):
                return planner.encode()
            return readonly.encode()

        with mock.patch.object(qualifier, "git_show", side_effect=fake_show):
            result = qualifier.verify_authority_surface()
        self.assertEqual(result["executor_post_count"], 1)
        self.assertFalse(result["planner_mutation_authority"])
        self.assertTrue(result["readonly_client_get_only"])

    def test_authority_surface_rejects_extra_post(self):
        executor = '''\nmethod="POST"\nmethod="POST"\n/actions/runs/{run_id}/cancel\n'''
        planner = "GitHubReadOnlyClient\n"
        readonly = 'method="GET"\n'
        with mock.patch.object(
            qualifier,
            "git_show",
            side_effect=[executor.encode(), planner.encode(), readonly.encode()],
        ):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.verify_authority_surface()

    def test_authority_surface_rejects_scope_widening(self):
        executor = '''\nmethod="POST"\n/actions/runs/{run_id}/cancel\n--all\n'''
        planner = "GitHubReadOnlyClient\n"
        readonly = 'method="GET"\n'
        with mock.patch.object(
            qualifier,
            "git_show",
            side_effect=[executor.encode(), planner.encode(), readonly.encode()],
        ):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.verify_authority_surface()

    def test_safe_extract_rejects_parent_traversal(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            archive = root / "bad.tar"
            info = tarfile.TarInfo("../escape")
            data = b"x"
            info.size = len(data)
            with tarfile.open(archive, "w") as tf:
                tf.addfile(info, io.BytesIO(data))
            with self.assertRaises(qualifier.QualificationError):
                qualifier.safe_extract_archive(archive, root / "out")

    def test_safe_extract_rejects_symlink_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            archive = root / "link.tar"
            info = tarfile.TarInfo("link")
            info.type = tarfile.SYMTYPE
            info.linkname = "target"
            with tarfile.open(archive, "w") as tf:
                tf.addfile(info)
            out = root / "out"
            out.mkdir()
            with self.assertRaises(qualifier.QualificationError):
                qualifier.safe_extract_archive(archive, out)

    def test_safe_extract_accepts_normal_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            archive = root / "ok.tar"
            info = tarfile.TarInfo("a/b.txt")
            data = b"ok"
            info.size = len(data)
            with tarfile.open(archive, "w") as tf:
                tf.addfile(info, io.BytesIO(data))
            out = root / "out"
            out.mkdir()
            observed = qualifier.safe_extract_archive(archive, out)
            self.assertEqual(observed, ["a/b.txt"])
            self.assertEqual((out / "a/b.txt").read_bytes(), data)

    def test_identity_failure_is_fail_closed(self):
        with mock.patch.object(qualifier, "commit_tree", return_value="wrong"):
            with self.assertRaises(qualifier.QualificationError):
                qualifier.verify_identity()

    def test_receipt_commitment_binds_receipt_body(self):
        body = {
            "schema": qualifier.SCHEMA,
            "executor": qualifier.EXECUTOR,
            "tests": [{"returncode": 0}],
        }
        first = qualifier.canonical_sha256(body)
        body["tests"][0]["returncode"] = 1
        second = qualifier.canonical_sha256(body)
        self.assertNotEqual(first, second)

    def test_python_test_execution_is_isolated(self):
        completed = mock.Mock(returncode=0, stdout=b"ok", stderr=b"")
        with mock.patch.object(qualifier.subprocess, "run", return_value=completed) as run:
            item = qualifier.run_python_test(pathlib.Path("/tmp"), "test.py")
        command = run.call_args.args[0]
        self.assertEqual(command[1:], ["-E", "-s", "-S", "-B", "test.py"])
        self.assertEqual(item["command"], ["python", "-E", "-s", "-S", "-B", "test.py"])

    def test_no_network_or_actions_mutation_client_in_qualifier_source(self):
        text = (HERE / "ci_gov_001d_exact_qualify.py").read_text()
        self.assertNotIn("api.github.com", text)
        self.assertNotIn("urllib.request", text)
        self.assertNotIn("requests.", text)
        self.assertNotIn("cancel_run(", text)
        self.assertNotIn("rerun", text.lower())


if __name__ == "__main__":
    unittest.main()
