#!/usr/bin/env python3
"""Fast source-contract preflight for the synthetic Logistics Commons UI.

This checks source structure only. It does not compile Rust, execute Rust tests,
build WASM, render a browser, or qualify the S0 simulator/verifier.
"""
from __future__ import annotations
import argparse
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path("mycelix-workspace/mycelix-commons/apps/leptos/src/pages/logistics.rs")
APP = Path("mycelix-workspace/mycelix-commons/apps/leptos/src/app.rs")
PAGES = Path("mycelix-workspace/mycelix-commons/apps/leptos/src/pages/mod.rs")
CSS = Path("mycelix-workspace/mycelix-commons/apps/leptos/style/main.css")
WORKFLOW = Path(".github/workflows/logistics-commons-ux.yml")
RUNBOOK = Path("docs/integral/logistics-ux-qualification.md")
LOCKFILE = Path("mycelix-workspace/mycelix-commons/apps/leptos/Cargo.lock")

EXPECTED_KINDS = {"Conflict": 3, "Pending": 1, "Stale": 1, "Evidence": 1, "Alias": 1}
EXPECTED_FILTERS = {"Attention", "Conflict", "Evidence", "All"}
REQUIRED_TESTS = {
    "conflict_fixture_contains_all_three_contenders_and_is_oversubscribed",
    "attention_filter_includes_pending_work_and_exceptions_but_not_aliases",
    "evidence_filter_includes_missing_receipt_and_stale_claims",
    "query_is_case_insensitive_and_searches_evidence_freshness_context_and_safe_steps",
    "search_and_filter_must_both_match",
    "fixture_ids_are_unique",
    "every_fixture_has_a_safe_step_and_reason",
    "next_safe_steps_respect_authority_and_evidence_boundaries",
    "queue_counts_stay_consistent_with_search_and_filter_results",
    "fixture_kind_partition_is_complete_and_explicit",
    "all_filter_includes_every_hand_authored_fixture",
}


def enum_variants(source: str, name: str) -> set[str]:
    match = re.search(rf"\benum\s+{re.escape(name)}\s*\{{(.*?)\}}", source, re.S)
    if not match:
        return set()
    return set(re.findall(r"^\s*([A-Z][A-Za-z0-9_]*)\s*,?\s*$", match.group(1), re.M))


def fixture_blocks(source: str) -> tuple[int, list[str]]:
    match = re.search(
        r"const\s+SAMPLE_ORDERS:\s*\[OrderFixture;\s*(\d+)\]\s*=\s*\[(.*?)\n\];",
        source, re.S,
    )
    if not match:
        return 0, []
    return int(match.group(1)), re.findall(r"OrderFixture\s*\{(.*?)\n    \},", match.group(2), re.S)


def field_value(block: str, name: str) -> str:
    match = re.search(rf"^\s*{re.escape(name)}:\s*(.*?)\s*,?\s*$", block, re.M)
    if not match:
        return ""
    value = match.group(1).strip()
    return value[1:-1] if len(value) >= 2 and value[0] == value[-1] == '"' else value


def explicit_filter_match(source: str) -> bool:
    parts = source.split("fn order_matches(", 1)
    if len(parts) != 2:
        return False
    body = parts[1].split("const SAMPLE_ORDERS", 1)[0]
    return all(f"QueueFilter::{name} =>" in body for name in EXPECTED_FILTERS) and not re.search(r"_\s*=>\s*true", body)


def pinned_action_counts(workflow: str) -> tuple[int, int]:
    """Return (fully SHA-pinned action references, all action references)."""
    references = re.findall(r"^\s*uses:\s*(\S+)", workflow, re.M)
    pinned = [
        reference for reference in references
        if re.fullmatch(r"[^@\s]+@[0-9a-f]{40}", reference)
    ]
    return len(pinned), len(references)


def all_actions_full_sha_pinned(workflow: str) -> bool:
    pinned, total = pinned_action_counts(workflow)
    return total >= 2 and pinned == total


def audit(source: str, app: str, pages: str, css: str, workflow: str, runbook_exists: bool) -> list[tuple[str, bool, str]]:
    results: list[tuple[str, bool, str]] = []

    def add(name: str, ok: bool, detail: str) -> None:
        results.append((name, bool(ok), detail))

    kinds = enum_variants(source, "FixtureKind")
    filters = enum_variants(source, "QueueFilter")
    declared, blocks = fixture_blocks(source)
    keys = ("id", "participant", "hub", "sku", "quantity", "operational", "evidence", "freshness", "scenario", "next_step", "next_step_reason", "kind")
    fixtures = [{key: field_value(block, key) for key in keys} for block in blocks]
    ids = [item["id"] for item in fixtures]
    counts = {kind: sum(item["kind"] == f"FixtureKind::{kind}" for item in fixtures) for kind in EXPECTED_KINDS}

    add("FixtureKind enum is exact", kinds == set(EXPECTED_KINDS), f"found={sorted(kinds)}")
    add("QueueFilter enum is exact", filters == EXPECTED_FILTERS, f"found={sorted(filters)}")
    add("fixture kind field is typed", bool(re.search(r"struct\s+OrderFixture\s*\{[\s\S]*?kind:\s*FixtureKind\s*,", source)), "OrderFixture.kind must use FixtureKind")
    add("declared and parsed fixture counts agree", declared == len(fixtures) == 7, f"declared={declared}, parsed={len(fixtures)}, expected=7")
    add("fixture IDs are unique", all(ids) and len(ids) == len(set(ids)), f"ids={ids}")
    missing = [item["id"] or f"fixture-{index}" for index, item in enumerate(fixtures) if any(not item.get(k, "").strip() for k in keys[1:])]
    add("each fixture has required evidence and guidance", not missing, f"missing={missing or 'none'}")
    add("fixture-kind distribution is exhaustive", counts == EXPECTED_KINDS, f"observed={counts}")

    conflicts = [item for item in fixtures if item["kind"] == "FixtureKind::Conflict"]
    supply = re.search(r"const\s+HOTSPOT_AVAILABLE_UNITS:\s*u32\s*=\s*(\d+)\s*;", source)
    conflict_ok, conflict_detail = False, "could not parse supply or contender quantities"
    if supply and conflicts:
        try:
            available = int(supply.group(1))
            requested = sum(int(item["quantity"]) for item in conflicts)
            same_resource = len({(item["hub"], item["sku"]) for item in conflicts}) == 1
            conflict_ok = same_resource and requested > available
            conflict_detail = f"requested={requested}, available={available}, same_resource={same_resource}"
        except ValueError:
            pass
    add("conflict fixture is oversubscribed", conflict_ok, conflict_detail)
    add("aliases are excluded from attention queue", "FixtureKind::Alias => false" in source and "fn needs_attention" in source, "must not treat duplicate replay aliases as actionable work")
    add("filter matching is explicit and fail-closed", explicit_filter_match(source), "all four variants require explicit match arms; wildcard-all is prohibited")
    count_controls = all(f'matching_order_count(&query.get(), QueueFilter::{name})' in source for name in EXPECTED_FILTERS)
    add("queue counts and summary share matcher", count_controls and "order_matches(order, query, filter)" in source and "matching_order_count(&query.get(), active_filter.get())" in source, "filter badges and result summary must reuse the same function")
    add("reset clears query and restores default filter", "query.set(String::new())" in source and "active_filter.set(QueueFilter::Attention)" in source, "both reactive controls must reset")
    add("no-results state is announced", 'class="logistics-empty-state"' in source and 'role="status"' in source and "No matching fixture records" in source, "empty queue state must be visible and accessible")

    test_area = source[source.find("#[cfg(test)]"):]
    declared_tests = set(re.findall(r"fn\s+([a-z][a-z0-9_]*)\s*\(\)\s*\{", test_area))
    missing_tests = sorted(REQUIRED_TESTS - declared_tests)
    add("required Rust test cases are present", not missing_tests, f"missing={missing_tests or 'none'}")
    add("nested route is registered and exported", 'path!("/transport/logistics")' in app and "pub use logistics::LogisticsWorkspacePage;" in pages, "route and page module export must agree")
    add("synthetic-only boundary is explicit", "SYNTHETIC UX PREVIEW" in source and "not live inventory" in source and "No mutation or booking action is available here." in source and "No actions enabled in synthetic preview." in source, "no live or mutation claim may be implied")
    add("basic accessibility styles are present", ":focus-visible" in css and "@media (max-width: 640px)" in css and "prefers-reduced-motion: reduce" in css, "focus, mobile layout, and reduced motion")

    expected_ref = "ref: ${{ github.event.pull_request.head.sha || github.sha }}"
    add("workflow checks out exact qualification SHA", expected_ref in workflow, "PR head SHA for pull requests; event SHA for main pushes")
    add("checkout SHA is asserted and credentials are not persisted", "Assert exact qualification subject" in workflow and "git rev-parse HEAD" in workflow and "persist-credentials: false" in workflow, "fail before build steps on SHA mismatch")
    pinned_count, action_count = pinned_action_counts(workflow)
    add(
        "every workflow action is full-SHA pinned",
        all_actions_full_sha_pinned(workflow),
        f"pinned={pinned_count}, uses={action_count}",
    )
    add("Rust version and WASM target are explicit", "toolchain: 1.99.0" in workflow and "targets: wasm32-unknown-unknown" in workflow, "do not use an ambient toolchain")
    source_step = workflow.find("Run dependency-free Logistics UX source contract")
    rust_step = workflow.find("Install Rust and WASM target")
    add("preflight runs before Rust installation", source_step >= 0 and source_step < rust_step and "python3 scripts/integral/verify_logistics_ux_contract.py" in workflow, "run the fast, dependency-free gate first")
    add("checker and runbook changes trigger workflow", "scripts/integral/verify_logistics_ux_contract.py" in workflow and "docs/integral/logistics-ux-qualification.md" in workflow, "process assets must be in both path filters")
    add("format, Rust test, and WASM gates remain", "rustfmt --edition 2024 --check" in workflow and "cargo test --manifest-path mycelix-workspace/mycelix-commons/apps/leptos/Cargo.toml" in workflow and "cargo check --manifest-path mycelix-workspace/mycelix-commons/apps/leptos/Cargo.toml --target wasm32-unknown-unknown" in workflow, "preflight must not replace compiler or runtime evidence")
    add("qualification runbook exists", runbook_exists, "docs/integral/logistics-ux-qualification.md")
    return results


class VerifierTests(unittest.TestCase):
    def test_enum_parser(self) -> None:
        self.assertEqual(enum_variants("enum F {\n A,\n B,\n}", "F"), {"A", "B"})

    def test_fixture_parser_and_field_reader(self) -> None:
        source = '''const SAMPLE_ORDERS: [OrderFixture; 2] = [
    OrderFixture {
        id: "LC-01",
        kind: FixtureKind::Alias,
    },
    OrderFixture {
        id: "LC-02",
        kind: FixtureKind::Conflict,
    },
];'''
        count, blocks = fixture_blocks(source)
        self.assertEqual(count, 2)
        self.assertEqual(len(blocks), 2)
        self.assertEqual(field_value(blocks[0], "id"), "LC-01")
        self.assertEqual(field_value(blocks[1], "kind"), "FixtureKind::Conflict")

    def test_filter_contract_rejects_missing_arm(self) -> None:
        good = """fn order_matches(x: X) -> bool {
 QueueFilter::Attention => true,
 QueueFilter::Conflict => true,
 QueueFilter::Evidence => true,
 QueueFilter::All => true,
}
const SAMPLE_ORDERS = [];"""
        self.assertTrue(explicit_filter_match(good))
        self.assertFalse(explicit_filter_match(good.replace("QueueFilter::Conflict => true,", "")))

    def test_filter_contract_rejects_wildcard_all(self) -> None:
        bad = """fn order_matches(x: X) -> bool {
 QueueFilter::Attention => true,
 QueueFilter::Conflict => true,
 QueueFilter::Evidence => true,
 QueueFilter::All => true,
 _ => true,
}
const SAMPLE_ORDERS = [];"""
        self.assertFalse(explicit_filter_match(bad))


    def test_action_pin_contract_rejects_unpinned_addition(self) -> None:
        digest = "a" * 40
        pinned = f"""steps:
  - name: checkout
    uses: actions/checkout@{digest}
  - name: rust
    uses: dtolnay/rust-toolchain@{digest}
"""
        self.assertTrue(all_actions_full_sha_pinned(pinned))
        unpinned = pinned + """  - name: cache
    uses: actions/cache@v4
"""
        malformed = pinned + """  - name: invalid
    uses: owner/action@not-a-sha
"""
        self.assertFalse(all_actions_full_sha_pinned(unpinned))
        self.assertFalse(all_actions_full_sha_pinned(malformed))


def run_self_tests() -> bool:
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(VerifierTests)
    result = unittest.TextTestRunner(stream=sys.stderr, verbosity=0).run(suite)
    if result.wasSuccessful():
        print(f"[PASS] preflight-tool self-tests ({result.testsRun} cases)")
    return result.wasSuccessful()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test-only", action="store_true", help="test the preflight parser without auditing the repository")
    args = parser.parse_args()
    if not run_self_tests():
        return 2
    if args.self_test_only:
        return 0
    try:
        source = (ROOT / SOURCE).read_text(encoding="utf-8")
        app = (ROOT / APP).read_text(encoding="utf-8")
        pages = (ROOT / PAGES).read_text(encoding="utf-8")
        css = (ROOT / CSS).read_text(encoding="utf-8")
        workflow = (ROOT / WORKFLOW).read_text(encoding="utf-8")
    except OSError as exc:
        print(f"[FAIL] required source files readable: {exc}")
        return 2

    results = audit(source, app, pages, css, workflow, (ROOT / RUNBOOK).is_file())
    failures = 0
    for name, passed, detail in results:
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}")
        failures += not passed

    if (ROOT / LOCKFILE).is_file():
        print(f"[INFO] Cargo.lock exists at {LOCKFILE}; review its digest and require --locked gates before claiming reproducibility.")
    else:
        print(f"[INCOMPLETE] Cargo.lock absent at {LOCKFILE}; dependency resolution is not locked.")

    if failures:
        print(f"SOURCE CONTRACT FAIL: {failures}/{len(results)} checks failed; this is not a Rust compiler result.")
        return 1
    print(f"SOURCE CONTRACT PASS: {len(results)} checks passed. This is not Rust-test, WASM, browser, S0, or production qualification.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
