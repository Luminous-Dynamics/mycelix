import importlib.util
import pathlib
import unittest
from datetime import datetime, timezone

SPEC = importlib.util.spec_from_file_location("census", str(pathlib.Path(__file__).with_name("ci_queue_census.py")))
census = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(census)


class QueueCensusTests(unittest.TestCase):
    def setUp(self):
        self.now = datetime(2026, 9, 15, 16, 0, tzinfo=timezone.utc)

    def test_summary_classifies_prs_and_superseded_heads(self):
        queued = [
            {
                "id": 1,
                "name": "A",
                "created_at": "2026-09-15T14:00:00Z",
                "head_sha": "old",
                "pull_requests": [{"number": 10}],
            },
            {
                "id": 2,
                "name": "A",
                "created_at": "2026-09-15T15:30:00Z",
                "head_sha": "same",
                "pull_requests": [{"number": 11}],
            },
            {
                "id": 3,
                "name": "B",
                "created_at": "2026-09-15T15:45:00Z",
                "head_sha": "push",
                "pull_requests": [],
            },
        ]
        in_progress = [
            {
                "id": 4,
                "name": "C",
                "created_at": "2026-09-15T15:59:00Z",
                "head_sha": "ip",
                "pull_requests": [{"number": 12}],
            }
        ]
        prs = {
            10: {"state": "open", "draft": True, "head": {"sha": "new"}},
            11: {"state": "open", "draft": False, "head": {"sha": "same"}},
            12: {"state": "closed", "draft": False, "head": {"sha": "ip"}},
        }
        result = census.summarize(queued, in_progress, prs, self.now)
        self.assertEqual(result["queued_count"], 3)
        self.assertEqual(result["in_progress_count"], 1)
        self.assertEqual(result["oldest_queued_age_seconds"], 7200)
        self.assertEqual(result["oldest_queued_run_id"], 1)
        self.assertEqual(result["queued_by_workflow"][0], {"workflow": "A", "count": 2})
        self.assertEqual(result["queued_pr_state_counts"]["draft"], 1)
        self.assertEqual(result["queued_pr_state_counts"]["ready"], 1)
        self.assertEqual(result["queued_pr_state_counts"]["non_pr"], 1)
        self.assertEqual(result["superseded_queued_count"], 1)
        self.assertEqual(result["superseded_queued_runs"][0]["run_id"], 1)

    def test_missing_pr_metadata_is_visible_not_guessed(self):
        queued = [{
            "id": 5,
            "name": "A",
            "created_at": "2026-09-15T15:00:00Z",
            "head_sha": "x",
            "pull_requests": [{"number": 99}],
        }]
        result = census.summarize(queued, [], {}, self.now)
        self.assertEqual(result["queued_pr_state_counts"]["unknown"], 1)
        self.assertEqual(result["unknown_pr_runs"], 1)
        self.assertEqual(result["superseded_queued_count"], 0)

    def test_duration_rendering(self):
        self.assertEqual(census.human_duration(None), "n/a")
        self.assertEqual(census.human_duration(90), "1m")
        self.assertEqual(census.human_duration(90061), "1d 1h 1m")

    def test_markdown_states_read_only_nonclaim(self):
        result = census.summarize([], [], {}, self.now)
        rendered = census.render_markdown(result)
        self.assertIn("Read-only telemetry", rendered)
        self.assertIn("does not cancel", rendered)


if __name__ == "__main__":
    unittest.main()
