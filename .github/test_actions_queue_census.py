import unittest
from datetime import datetime, timezone

import actions_queue_census as census

NOW = datetime(2026, 10, 10, 20, 0, 0, tzinfo=timezone.utc)


def make_run(run_id, created_at, *, name="Mycelix CI", sha="abc", pr=10):
    return {
        "id": run_id, "name": name, "run_number": run_id,
        "workflow_id": 123, "path": ".github/workflows/ci.yml",
        "event": "pull_request", "status": "queued", "conclusion": None,
        "head_branch": "feature/test", "head_sha": sha,
        "pull_requests": [{"number": pr}], "created_at": created_at,
        "updated_at": created_at,
        "html_url": f"https://github.com/Luminous-Dynamics/mycelix/actions/runs/{run_id}",
    }


class QueueCensusTests(unittest.TestCase):
    def test_pagination_flattens_and_deduplicates(self):
        first = make_run(1, "2026-10-10T19:00:00Z")
        second = make_run(2, "2026-10-10T19:30:00Z", pr=11)
        runs, total = census.flatten_pages([
            {"total_count": 2, "workflow_runs": [first]},
            {"total_count": 2, "workflow_runs": [first, second]},
        ], "workflow_runs")
        self.assertEqual(total, 2)
        self.assertEqual([run["id"] for run in runs], [1, 2])

    def test_conflicting_duplicate_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "conflicting duplicate"):
            census.flatten_pages([
                {"total_count": 1, "workflow_runs": [make_run(1, "2026-10-10T19:00:00Z", sha="a")]},
                {"total_count": 1, "workflow_runs": [make_run(1, "2026-10-10T19:00:00Z", sha="b")]},
            ], "workflow_runs")

    def test_api_total_remains_distinct_from_fetched_count(self):
        summary, _ = census.summarize_status({
            "total_count": 3,
            "workflow_runs": [make_run(1, "2026-10-10T19:00:00Z")],
        }, NOW)
        self.assertEqual(summary["api_total_count"], 3)
        self.assertEqual(summary["fetched_unique_count"], 1)
        self.assertFalse(summary["pagination_complete_by_count"])

    def test_age_oldest_newest_and_group_counts(self):
        summary, _ = census.summarize_status({
            "total_count": 3,
            "workflow_runs": [
                make_run(1, "2026-10-10T18:00:00Z", name="Audit", pr=11),
                make_run(2, "2026-10-10T19:00:00Z", name="Mycelix CI", pr=12),
                make_run(3, "2026-10-10T19:30:00Z", name="Audit", pr=11),
            ],
        }, NOW)
        self.assertEqual(summary["oldest_age_hours"], 2.0)
        self.assertEqual(summary["oldest_run"]["run_id"], 1)
        self.assertEqual(summary["newest_run"]["run_id"], 3)
        self.assertEqual(summary["by_workflow"]["Audit"], 2)
        self.assertEqual(summary["by_pull_request"]["11"], 2)

    def test_api_error_is_not_an_empty_queue(self):
        with self.assertRaisesRegex(ValueError, "API returned an error"):
            census.flatten_pages({"message": "Bad credentials"}, "workflow_runs")

    def test_job_metadata_preserves_unassigned_runner(self):
        jobs, total = census.flatten_pages([
            {"total_count": 1, "jobs": [{"id": 77, "status": "queued",
                                         "runner_name": None, "labels": ["ubuntu-latest"]}]},
            {"total_count": 1, "jobs": [{"id": 77, "status": "queued",
                                         "runner_name": None, "labels": ["ubuntu-latest"]}]},
        ], "jobs")
        self.assertEqual(total, 1)
        self.assertEqual(len(jobs), 1)
        self.assertIsNone(jobs[0]["runner_name"])

    def test_census_is_read_only_and_limits_interpretation(self):
        empty = {"total_count": 0, "workflow_runs": []}
        result = census.build_census("Luminous-Dynamics/mycelix", empty, empty, NOW)
        self.assertTrue(result["read_only"])
        self.assertEqual(result["statuses"]["queued"]["api_total_count"], 0)
        self.assertIn("does not establish root cause", result["interpretation_boundary"])

    def test_census_preserves_exact_queued_and_active_run_manifests(self):
        queued = {"total_count": 2, "workflow_runs": [
            make_run(1, "2026-10-10T18:00:00Z", sha="sha-one"),
            make_run(2, "2026-10-10T19:00:00Z", sha="sha-two", pr=11),
        ]}
        active = make_run(3, "2026-10-10T19:30:00Z", sha="sha-three", pr=12)
        active["status"] = "in_progress"
        result = census.build_census("Luminous-Dynamics/mycelix", queued,
                                     {"total_count": 1, "workflow_runs": [active]}, NOW)
        self.assertEqual([run["head_sha"] for run in result["queued_run_manifest"]],
                         ["sha-one", "sha-two"])
        self.assertEqual(result["in_progress_run_manifest"][0]["run_id"], 3)
        self.assertEqual(result["queued_run_manifest"][0]["workflow_id"], 123)
        self.assertEqual(result["queued_run_manifest"][0]["workflow_path"],
                         ".github/workflows/ci.yml")

    def test_job_sampling_is_bounded(self):
        runs = [
            census.run_identity(make_run(i, f"2026-10-10T{10 + i:02d}:00:00Z",
                                         name=f"Workflow {i}", pr=i))
            for i in range(1, 10)
        ]
        sampled = census.choose_samples(runs, 4)
        self.assertLessEqual(len(sampled), 4)
        self.assertEqual(len({str(run["run_id"]) for run in sampled}), len(sampled))


if __name__ == "__main__":
    unittest.main()
