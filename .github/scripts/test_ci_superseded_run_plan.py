import importlib.util
import pathlib
import sys
import unittest
from datetime import datetime, timezone
from unittest import mock

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
REPO = "Luminous-Dynamics/mycelix"

SPEC = importlib.util.spec_from_file_location("planner", str(HERE / "ci_superseded_run_plan.py"))
planner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(planner)


def run(run_id=1, head="old", *, path=".github/workflows/ci.yml", event="pull_request", status="queued", prs=(10,), run_attempt=1, name="Mycelix CI"):
    return {
        "id": run_id,
        "run_attempt": run_attempt,
        "name": name,
        "workflow_id": 100,
        "path": path,
        "event": event,
        "status": status,
        "head_sha": head,
        "created_at": f"2026-09-18T12:00:{run_id:02d}Z",
        "pull_requests": [{"number": number} for number in prs],
    }


def pr(head, *, state="open", draft=True):
    return {"state": state, "draft": draft, "head": {"sha": head}}


class SupersededRunPlanTests(unittest.TestCase):
    def setUp(self):
        self.now = datetime(2026, 9, 18, 14, 0, tzinfo=timezone.utc)

    def classify(self, item, prs):
        return planner.classify_run(item, prs, REPO)

    def test_single_pr_superseded_is_eligible(self):
        entry = self.classify(run(), {10: pr("new")})
        self.assertEqual(entry["classification"], planner.ELIGIBLE)
        self.assertTrue(entry["eligible_for_cancellation_plan"])
        self.assertEqual(entry["repository"], REPO)
        self.assertEqual(entry["run_attempt"], 1)

    def test_current_head_is_not_eligible(self):
        entry = self.classify(run(), {10: pr("old")})
        self.assertEqual(entry["classification"], planner.CURRENT)
        self.assertFalse(entry["eligible_for_cancellation_plan"])

    def test_multi_pr_requires_every_open_association_to_be_superseded(self):
        item = run(prs=(10, 11))
        self.assertEqual(self.classify(item, {10: pr("new-a"), 11: pr("old")})["classification"], planner.CURRENT)
        self.assertEqual(self.classify(item, {10: pr("new-a"), 11: pr("new-b")})["classification"], planner.ELIGIBLE)

    def test_closed_missing_and_malformed_associations_fail_closed(self):
        self.assertEqual(self.classify(run(prs=(10, 11)), {10: pr("new-a"), 11: pr("new-b", state="closed")})["classification"], planner.CLOSED)
        self.assertEqual(self.classify(run(prs=(10, 11)), {10: pr("new")})["classification"], planner.UNKNOWN)
        item = run()
        item["pull_requests"] = [{"number": 10}, {"id": 999}]
        self.assertEqual(self.classify(item, {10: pr("new")})["classification"], planner.UNKNOWN)

    def test_unknown_precedence_is_order_independent(self):
        a = self.classify(run(prs=(10, 11)), {10: pr("new", state="closed")})
        b = self.classify(run(prs=(11, 10)), {10: pr("new", state="closed")})
        self.assertEqual(a["classification"], planner.UNKNOWN)
        self.assertEqual(b["classification"], planner.UNKNOWN)

    def test_scope_rules(self):
        self.assertEqual(self.classify(run(path=".github/workflows/amsap-004a-subject-topology.yml"), {10: pr("new")})["classification"], planner.OUT_OF_SCOPE)
        self.assertEqual(self.classify(run(event="push", prs=()), {})["classification"], planner.NON_PR)
        self.assertEqual(self.classify(run(status="in_progress"), {10: pr("new")})["classification"], planner.STATUS_OUT_OF_SCOPE)

    def test_draft_does_not_change_supersession(self):
        self.assertEqual(self.classify(run(), {10: pr("new", draft=True)})["classification"], planner.ELIGIBLE)
        self.assertEqual(self.classify(run(), {10: pr("new", draft=False)})["classification"], planner.ELIGIBLE)

    def test_entry_commitment_properties(self):
        prs = {10: pr("new-a"), 11: pr("new-b")}
        a = self.classify(run(prs=(11, 10)), prs)
        b = self.classify(run(prs=(10, 11)), prs)
        self.assertEqual(a["plan_entry_commitment"], b["plan_entry_commitment"])
        c = planner.classify_run(run(), {10: pr("new")}, "Other/repo")
        d = planner.classify_run(run(run_attempt=2), {10: pr("new")}, REPO)
        e = planner.classify_run(run(), {10: pr("new")}, REPO)
        self.assertNotEqual(e["plan_entry_commitment"], c["plan_entry_commitment"])
        self.assertNotEqual(e["plan_entry_commitment"], d["plan_entry_commitment"])

    def test_missing_run_attempt_fails_closed(self):
        item = run()
        item.pop("run_attempt")
        self.assertEqual(self.classify(item, {10: pr("new")})["classification"], planner.UNKNOWN)

    def test_plan_scope_expiry_and_counts(self):
        plan = planner.build_plan(
            [run(1, prs=(10,)), run(2, head="same", prs=(11,))],
            {10: pr("new"), 11: pr("same")},
            self.now,
            REPO,
        )
        self.assertEqual(plan["repository"], REPO)
        self.assertEqual(plan["entry_count"], 2)
        self.assertEqual(plan["eligible_count"], 1)
        self.assertEqual([e["run_id"] for e in plan["eligible_entries"]], [1])
        self.assertFalse(plan["scope"]["mutation_authority"])
        self.assertFalse(plan["scope"]["runtime_workflow_override_allowed"])
        self.assertFalse(plan["scope"]["runtime_time_override_allowed"])
        self.assertTrue(plan["scope"]["envelope_contains_only_in_scope_runs"])
        self.assertTrue(plan["scope"]["executor_requires_full_plan_envelope"])
        self.assertEqual(plan["scope"]["plan_ttl_seconds"], planner.PLAN_TTL_SECONDS)
        self.assertEqual(plan["observed_at"], "2026-09-18T14:00:00Z")
        self.assertEqual(plan["expires_at"], "2026-09-18T14:15:00Z")
        self.assertEqual(len(plan["plan_commitment"]), 64)

    def test_plan_commitment_changes_with_time_and_noneligible_in_scope_entries(self):
        a = planner.build_plan([], {}, self.now, REPO)
        later = datetime(2026, 9, 18, 14, 1, tzinfo=timezone.utc)
        b = planner.build_plan([], {}, later, REPO)
        self.assertNotEqual(a["plan_commitment"], b["plan_commitment"])

        current = run(head="same")
        p1 = planner.build_plan([current], {10: pr("same")}, self.now, REPO)
        current2 = dict(current)
        current2["name"] = "Changed display name"
        p2 = planner.build_plan([current2], {10: pr("same")}, self.now, REPO)
        self.assertEqual(p1["eligible_count"], 0)
        self.assertEqual(p2["eligible_count"], 0)
        self.assertNotEqual(p1["plan_commitment"], p2["plan_commitment"])

    def test_unrelated_workflow_traffic_does_not_churn_plan_commitment(self):
        broad = run(1, prs=(10,))
        base = planner.build_plan([broad], {10: pr("new")}, self.now, REPO)
        noisy = planner.build_plan(
            [
                broad,
                run(2, path=".github/workflows/amsap-004a-subject-topology.yml", prs=(900,), name="AMSAP-004A Subject Topology"),
                run(3, path=".github/workflows/forge.yml", prs=(901,), name="Mycelix Forge CI"),
                run(4, event="push", prs=()),
                run(5, status="in_progress", prs=(902,)),
            ],
            {10: pr("new"), 900: pr("other"), 901: pr("other"), 902: pr("other")},
            self.now,
            REPO,
        )
        self.assertEqual(base["entry_count"], 1)
        self.assertEqual(noisy["entry_count"], 1)
        self.assertEqual(base["plan_commitment"], noisy["plan_commitment"])
        self.assertEqual([e["run_id"] for e in noisy["all_entries"]], [1])

    def test_malformed_in_scope_run_remains_in_envelope_and_fails_closed(self):
        item = run()
        item["pull_requests"] = [{"number": 10}, {"id": 999}]
        plan = planner.build_plan([item], {10: pr("new")}, self.now, REPO)
        self.assertEqual(plan["entry_count"], 1)
        self.assertEqual(plan["eligible_count"], 0)
        self.assertEqual(plan["all_entries"][0]["classification"], planner.UNKNOWN)

    def test_cli_cannot_override_workflow_scope_or_live_time(self):
        with mock.patch.object(sys, "argv", ["planner"]):
            args = planner.parse_args()
        self.assertFalse(hasattr(args, "workflow_path"))
        self.assertFalse(hasattr(args, "now"))

    def test_lookup_budget_ignores_out_of_scope_workflows(self):
        queued = [
            run(1, prs=(10,)),
            run(2, path=".github/workflows/amsap-004a-subject-topology.yml", prs=(900, 901)),
            run(3, event="push", prs=(902,)),
            run(4, status="in_progress", prs=(903,)),
        ]
        self.assertEqual(planner.pr_numbers_for_lookup(queued), [10])
        self.assertEqual([item["id"] for item in planner.runs_for_plan(queued)], [1])

    def test_markdown_states_no_authority_and_expiry(self):
        rendered = planner.render_markdown(planner.build_plan([], {}, self.now, REPO))
        self.assertIn("Read-only plan", rendered)
        self.assertIn("not cancellation authority", rendered)
        self.assertIn("Expires", rendered)
        self.assertIn(REPO, rendered)


if __name__ == "__main__":
    unittest.main()
