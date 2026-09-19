import importlib.util
import json
import pathlib
import sys
import unittest
from datetime import datetime, timedelta, timezone
from unittest import mock

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
REPO = "Luminous-Dynamics/mycelix"
NOW = datetime(2026, 9, 19, 15, 0, tzinfo=timezone.utc)


def load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


planner = load("ci_superseded_run_plan")
executor = load("ci_superseded_run_execute")


def run(run_id=1, head="old", prs=(10,), status="queued", attempt=1):
    return {
        "id": run_id,
        "run_attempt": attempt,
        "name": "Mycelix CI",
        "workflow_id": 100,
        "path": ".github/workflows/ci.yml",
        "event": "pull_request",
        "status": status,
        "head_sha": head,
        "created_at": f"2026-09-19T14:59:{run_id:02d}Z",
        "pull_requests": [{"number": number} for number in prs],
    }


def pr(head, state="open", draft=True):
    return {"state": state, "draft": draft, "head": {"sha": head}}


def make_plan(items=None, prs=None):
    items = items or [run()]
    prs = prs or {10: pr("new")}
    return planner.build_plan(items, prs, NOW, REPO)


class FakeClient:
    def __init__(self, runs, prs, cancel_status=202):
        self.runs = {item["id"]: item for item in runs}
        self.prs = prs
        self.cancel_status = cancel_status
        self.cancelled = []

    def get_run(self, run_id):
        return self.runs[run_id]

    def get_pull_request(self, number):
        return self.prs.get(number)

    def cancel_run(self, run_id):
        self.cancelled.append(run_id)
        return self.cancel_status


class ExecutorTests(unittest.TestCase):
    def execute(self, plan, client, ids=(1,), apply=False, now=NOW, sink=None):
        return executor.execute(
            plan,
            list(ids),
            client,
            REPO,
            now,
            apply=apply,
            authority_ref="operator-ticket:#1610-pilot",
            confirm_plan=plan["plan_commitment"],
            receipt_sink=sink,
        )

    def assert_stopped(self, out, client):
        self.assertEqual(out["outcome"], "StoppedOnRefusal")
        self.assertEqual(client.cancelled, [])

    def test_valid_dry_run_never_mutates(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("new")})
        out = self.execute(plan, client)
        self.assertEqual(out["outcome"], "DryRunComplete")
        self.assertEqual(out["receipts"][0]["outcome"], "DryRunEligibleNoMutation")
        self.assertEqual(client.cancelled, [])

    def test_apply_mutates_only_selected_run(self):
        items = [run(1, prs=(10,)), run(2, head="old2", prs=(11,))]
        prs = {10: pr("new"), 11: pr("new2")}
        plan = make_plan(items, prs)
        client = FakeClient(items, prs)
        out = self.execute(plan, client, ids=(2,), apply=True)
        self.assertEqual(client.cancelled, [2])
        self.assertEqual(out["outcome"], "ApplyRequestsAccepted")
        self.assertEqual(out["receipts"][0]["outcome"], "CancelRequestAccepted")

    def test_batch_bound_is_five(self):
        items = [run(i, head=f"old{i}", prs=(i,)) for i in range(1, 7)]
        prs = {i: pr(f"new{i}") for i in range(1, 7)}
        plan = make_plan(items, prs)
        client = FakeClient(items, prs)
        with self.assertRaises(executor.Refusal):
            self.execute(plan, client, ids=tuple(range(1, 7)), apply=True)
        self.assertEqual(client.cancelled, [])

    def test_expired_plan_refuses_before_mutation(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("new")})
        with self.assertRaises(executor.Refusal):
            self.execute(plan, client, now=NOW + timedelta(seconds=901), apply=True)
        self.assertEqual(client.cancelled, [])

    def test_plan_entry_tamper_refuses(self):
        plan = make_plan()
        plan["all_entries"][0]["queued_head_sha"] = "evil"
        with self.assertRaises(executor.Refusal):
            self.execute(plan, FakeClient([run()], {10: pr("new")}))

    def test_wrong_plan_confirmation_refuses(self):
        plan = make_plan()
        with self.assertRaises(executor.Refusal):
            executor.execute(
                plan, [1], FakeClient([run()], {10: pr("new")}), REPO, NOW,
                apply=False, authority_ref="ticket", confirm_plan="0" * 64,
            )

    def test_noneligible_selection_refuses(self):
        plan = make_plan([run()], {10: pr("old")})
        with self.assertRaises(executor.Refusal):
            self.execute(plan, FakeClient([run()], {10: pr("old")}))

    def test_live_head_change_refuses(self):
        plan = make_plan()
        client = FakeClient([run(head="different")], {10: pr("new")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_run_attempt_change_refuses(self):
        plan = make_plan()
        client = FakeClient([run(attempt=2)], {10: pr("new")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_pr_advanced_again_requires_replan(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("newer")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_pr_returns_to_queued_head_refuses(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("old")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_closed_pr_refuses(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("new", state="closed")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_changed_pr_association_set_refuses(self):
        plan = make_plan()
        client = FakeClient([run(prs=(10, 11))], {10: pr("new"), 11: pr("new2")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_completed_run_refuses(self):
        plan = make_plan()
        client = FakeClient([run(status="completed")], {10: pr("new")})
        self.assert_stopped(self.execute(plan, client, apply=True), client)

    def test_in_progress_same_identity_is_cancellable(self):
        plan = make_plan()
        client = FakeClient([run(status="in_progress")], {10: pr("new")})
        out = self.execute(plan, client, apply=True)
        self.assertEqual(client.cancelled, [1])
        self.assertEqual(out["receipts"][0]["pre_cancel_status"], "in_progress")

    def test_empty_authority_ref_refuses(self):
        plan = make_plan()
        with self.assertRaises(executor.Refusal):
            executor.execute(
                plan, [1], FakeClient([run()], {10: pr("new")}), REPO, NOW,
                apply=False, authority_ref="", confirm_plan=plan["plan_commitment"],
            )

    def test_receipt_sink_records_intent_before_mutation(self):
        plan = make_plan()
        client = FakeClient([run()], {10: pr("new")})
        snapshots = []
        out = self.execute(
            plan, client, apply=True,
            sink=lambda payload: snapshots.append(json.loads(json.dumps(payload))),
        )
        self.assertTrue(any(
            snap["receipts"] and snap["receipts"][-1]["outcome"] == "MutationIntentRecorded"
            for snap in snapshots
        ))
        self.assertEqual(out["receipts"][0]["outcome"], "CancelRequestAccepted")

    def test_transport_uncertainty_is_recorded_and_stops(self):
        class Boom(FakeClient):
            def cancel_run(self, run_id):
                self.cancelled.append(run_id)
                raise TimeoutError("uncertain")

        items = [run(1, prs=(10,)), run(2, head="old2", prs=(11,))]
        prs = {10: pr("new"), 11: pr("new2")}
        plan = make_plan(items, prs)
        client = Boom(items, prs)
        out = self.execute(plan, client, ids=(1, 2), apply=True)
        self.assertEqual(out["outcome"], "StoppedOnMutationError")
        self.assertEqual(out["receipts"][0]["outcome"], "CancelOutcomeUnknown")
        self.assertEqual(client.cancelled, [1])

    def test_late_refusal_stops_without_mutating_later_runs(self):
        items = [run(1, prs=(10,)), run(2, head="old2", prs=(11,))]
        prs = {10: pr("new"), 11: pr("new2")}
        plan = make_plan(items, prs)
        live = [run(1, prs=(10,)), run(2, head="wrong", prs=(11,))]
        client = FakeClient(live, prs)
        out = self.execute(plan, client, ids=(1, 2), apply=True)
        self.assertEqual(out["outcome"], "StoppedOnRefusal")
        self.assertEqual(client.cancelled, [1])
        self.assertEqual(out["receipts"][1]["outcome"], "RefusedAtRevalidation")

    def test_receipt_commitment_is_present(self):
        plan = make_plan()
        out = self.execute(plan, FakeClient([run()], {10: pr("new")}))
        self.assertEqual(len(out["receipts"][0]["receipt_commitment"]), 64)

    def test_source_has_exactly_one_post_mutation_and_it_is_cancel(self):
        text = (HERE / "ci_superseded_run_execute.py").read_text()
        self.assertEqual(text.count('method="POST"'), 1)
        self.assertIn('/actions/runs/{run_id}/cancel', text)
        self.assertNotIn('method="DELETE"', text)
        self.assertNotIn('method="PATCH"', text)
        self.assertNotIn('method="PUT"', text)

    def test_cli_has_no_all_or_scope_override(self):
        with mock.patch.object(sys, "argv", [
            "executor", "plan.json", "--confirm-plan", "abc", "--authority-ref", "ticket"
        ]):
            args = executor.parse_args()
        self.assertFalse(hasattr(args, "all"))
        self.assertFalse(hasattr(args, "workflow_path"))
        self.assertFalse(hasattr(args, "now"))


if __name__ == "__main__":
    unittest.main()
