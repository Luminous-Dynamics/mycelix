import copy
import importlib.util
import json
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("check_source_drift.py")
spec = importlib.util.spec_from_file_location("check_source_drift", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


def manifest(*sources):
    return {"profile": "test", "sources": list(sources)}


def blob(source_id="a", sha="a" * 40, status="draft"):
    return {
        "source_id": source_id,
        "status": status,
        "identity": {"kind": "github_blob", "blob_sha": sha},
    }


def web(source_id="web", token="observed-1", status="draft"):
    return {
        "source_id": source_id,
        "status": status,
        "identity": {
            "kind": "web_observation",
            "observation_token": token,
            "content_commitment": None,
        },
    }


class SourceDriftTests(unittest.TestCase):
    def test_exact_blob_is_unchanged(self):
        doc = manifest(blob())
        report = mod.compare(doc, copy.deepcopy(doc))
        self.assertFalse(report["requires_review"])
        self.assertEqual(report["counts"], {"unchanged": 1})

    def test_changed_blob_requires_review(self):
        base = manifest(blob())
        observed = manifest(blob(sha="b" * 40))
        report = mod.compare(base, observed)
        self.assertTrue(report["requires_review"])
        self.assertEqual(report["sources"][0]["classification"], "changed_generation")

    def test_status_promotion_requires_review_even_when_blob_is_same(self):
        base = manifest(blob(status="draft"))
        observed = manifest(blob(status="ratified"))
        report = mod.compare(base, observed)
        self.assertTrue(report["requires_review"])
        self.assertEqual(report["sources"][0]["classification"], "status_changed")

    def test_missing_and_new_sources_require_review(self):
        base = manifest(blob("old"))
        observed = manifest(blob("new"))
        report = mod.compare(base, observed)
        self.assertTrue(report["requires_review"])
        classes = {row["classification"] for row in report["sources"]}
        self.assertEqual(classes, {"missing_source", "new_source"})

    def test_web_observation_never_becomes_exact_by_token_equality(self):
        doc = manifest(web())
        report = mod.compare(doc, copy.deepcopy(doc))
        self.assertFalse(report["requires_review"])
        self.assertEqual(
            report["sources"][0]["classification"], "unverifiable_web_observation"
        )

    def test_web_observation_token_change_requires_review(self):
        base = manifest(web(token="observed-1"))
        observed = manifest(web(token="observed-2"))
        report = mod.compare(base, observed)
        self.assertTrue(report["requires_review"])
        self.assertEqual(report["sources"][0]["classification"], "changed_generation")

    def test_duplicate_source_ids_are_rejected(self):
        payload = manifest(blob("dup"), blob("dup", sha="b" * 40))
        path = Path(self.id().replace(".", "_") + ".json")
        try:
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate source_id"):
                mod.load_manifest(path)
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
