#!/usr/bin/env python3
"""Independent controls for bounded AAT capability attenuation and invocation rules."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import aat_capability_subsumption as aat  # noqa: E402

FROZEN_MAX_DEPTH = 32
FROZEN_MAX_NODES = 512
FROZEN_MAX_CLAUSES = 128
FROZEN_MAX_TOOLS = 256
FROZEN_MAX_ARGUMENT_KEYS = 64
FROZEN_MAX_CONSTRAINT_VALUE_DEPTH = 32
FROZEN_MAX_CONSTRAINT_VALUE_NODES = 512
FROZEN_MAX_TOOL_NAME_BYTES = 256
FROZEN_MAX_INVOCATION_VALUE_DEPTH = 32
FROZEN_MAX_INVOCATION_VALUE_NODES = 4096


def exact(value: Any) -> dict[str, Any]:
    return {"constraint_type": "exact", "value": value}


def range_c(**kwargs: Any) -> dict[str, Any]:
    return {"constraint_type": "range", **kwargs}


def one_of(*values: Any) -> dict[str, Any]:
    return {"constraint_type": "one_of", "values": list(values)}


def not_one_of(*values: Any) -> dict[str, Any]:
    return {"constraint_type": "not_one_of", "excluded": list(values)}


def contains(*values: Any) -> dict[str, Any]:
    return {"constraint_type": "contains", "required": list(values)}


def subset(*values: Any) -> dict[str, Any]:
    return {"constraint_type": "subset", "allowed": list(values)}


def wildcard() -> dict[str, Any]:
    return {"constraint_type": "wildcard"}


def all_c(*constraints: dict[str, Any]) -> dict[str, Any]:
    return {"constraint_type": "all", "constraints": list(constraints)}


def any_c(*constraints: dict[str, Any]) -> dict[str, Any]:
    return {"constraint_type": "any", "constraints": list(constraints)}


def aat_details(tools: dict[str, Any], *extra: dict[str, Any]) -> list[dict[str, Any]]:
    return [{"type": "attenuating_agent_token", "tools": tools}, *extra]


def tools_for(constraint: dict[str, Any], tool: str = "read_file",
              argument: str = "path") -> dict[str, Any]:
    return {tool: {argument: constraint}}


def finite_values() -> list[Any]:
    return [
        None, False, True, -1, 0, 1, 2, 2.5, 3, 10,
        "a", "b", "c", "x",
        [], [1], [1, 2], ["a"], ["a", "b"], ["c"], {},
        {"k": "v"}, {"k": "x"},
    ]


def strict_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        # JSON integer and floating values are the same JSON number category,
        # but booleans are never numbers for exact-value comparison.
        if type(left) in (int, float) and type(right) in (int, float):
            return left == right
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(strict_equal(left[k], right[k]) for k in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(strict_equal(a, b) for a, b in zip(left, right))
    return left == right


def contains_value(values: list[Any], value: Any) -> bool:
    return any(strict_equal(x, value) for x in values)


def independent_accepts(raw: dict[str, Any], value: Any) -> bool:
    """Independent raw-JSON runtime predicate; doesn't call aat.constraint_accepts."""
    kind = raw["constraint_type"]
    if kind == "exact":
        return strict_equal(value, raw["value"])
    if kind == "range":
        if type(value) not in (int, float) or (type(value) is float and not math.isfinite(value)):
            return False
        lo, hi = raw.get("min"), raw.get("max")
        if lo is not None and (value < lo or (value == lo and raw.get("min_inclusive", True) is False)):
            return False
        if hi is not None and (value > hi or (value == hi and raw.get("max_inclusive", True) is False)):
            return False
        return True
    if kind == "one_of":
        return contains_value(raw["values"], value)
    if kind == "not_one_of":
        return not contains_value(raw["excluded"], value)
    if kind == "contains":
        return isinstance(value, list) and all(contains_value(value, x) for x in raw["required"])
    if kind == "subset":
        return isinstance(value, list) and all(contains_value(raw["allowed"], x) for x in value)
    if kind == "wildcard":
        return True
    if kind == "all":
        return all(independent_accepts(child, value) for child in raw["constraints"])
    if kind == "any":
        return any(independent_accepts(child, value) for child in raw["constraints"])
    raise ValueError("unknown constraint type")


def denotation(raw: dict[str, Any]) -> set[str]:
    return {json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
            for value in finite_values() if independent_accepts(raw, value)}


def independently_sound_subsumption(derived: dict[str, Any], parent: dict[str, Any]) -> bool:
    return denotation(derived).issubset(denotation(parent))


def independently_bounded_json_value(value: Any, *, max_depth: int, max_nodes: int) -> bool:
    pending: list[tuple[Any, int]] = [(value, 1)]
    nodes = 0
    while pending:
        current, depth = pending.pop()
        nodes += 1
        if nodes > max_nodes or depth > max_depth:
            return False
        if current is None or type(current) in (bool, int, str):
            continue
        if type(current) is float:
            if not math.isfinite(current):
                return False
            continue
        if isinstance(current, list):
            pending.extend((item, depth + 1) for item in current)
            continue
        if isinstance(current, dict):
            if any(not isinstance(key, str) for key in current):
                return False
            pending.extend((item, depth + 1) for item in current.values())
            continue
        return False
    return True


def independently_invocation_check(tools: dict[str, Any], tool: str, args: dict[str, Any]) -> bool:
    if not independently_bounded_json_value(
        args, max_depth=FROZEN_MAX_INVOCATION_VALUE_DEPTH,
        max_nodes=FROZEN_MAX_INVOCATION_VALUE_NODES,
    ):
        return False
    if tool not in tools:
        return False
    constraints = tools[tool]
    if constraints and set(args) != set(constraints):
        return False
    return all(independent_accepts(constraint, args[name]) for name, constraint in constraints.items())


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def subsumption_cases() -> list[tuple[str, dict[str, Any], dict[str, Any], bool]]:
    return [
        ("exact-identical", exact("a"), exact("a"), True),
        ("exact-different", exact("b"), exact("a"), False),
        ("exact-within-range", exact(2), range_c(min=1, max=3), True),
        ("exact-on-exclusive-bound", exact(3), range_c(max=3, max_inclusive=False), False),
        ("exact-in-one-of", exact("a"), one_of("a", "b"), True),
        ("exact-outside-one-of", exact("c"), one_of("a", "b"), False),
        ("range-narrower", range_c(min=1, max=4), range_c(min=0, max=5), True),
        ("range-lower-bound-expanded", range_c(min=0, max=5), range_c(min=1, max=5), False),
        ("range-tightens-inclusive-min", range_c(min=1, min_inclusive=False, max=5), range_c(min=1, max=5), True),
        ("range-reopens-exclusive-min", range_c(min=1, max=5), range_c(min=1, min_inclusive=False, max=5), False),
        ("range-tightens-exclusive-max", range_c(min=0, max=5, max_inclusive=False), range_c(min=0, max=5), True),
        ("range-reopens-exclusive-max", range_c(min=0, max=5), range_c(min=0, max=5, max_inclusive=False), False),
        ("one-of-shrinks", one_of("a", "b"), one_of("a", "b", "c"), True),
        ("one-of-expands", one_of("a", "c"), one_of("a", "b"), False),
        ("not-one-of-adds-exclusion", not_one_of("a", "b"), not_one_of("a"), True),
        ("not-one-of-drops-exclusion", not_one_of("a"), not_one_of("a", "b"), False),
        ("contains-adds-required-item", contains("a", "b"), contains("a"), True),
        ("contains-drops-required-item", contains("a"), contains("a", "b"), False),
        ("subset-shrinks-allowed", subset("a"), subset("a", "b"), True),
        ("subset-expands-allowed", subset("a", "b"), subset("a"), False),
        ("wildcard-same", wildcard(), wildcard(), True),
        ("exact-narrows-wildcard", exact("a"), wildcard(), True),
        ("wildcard-cannot-widen-exact", wildcard(), exact("a"), False),
        ("all-one-to-one-reordered", all_c(exact("a"), one_of(1, 2)), all_c(one_of(1, 2, 3), exact("a")), True),
        ("all-parent-obligation-dropped", all_c(exact("a")), all_c(), False),
        ("all-child-adds-constraint", all_c(exact("a"), exact("b")), all_c(exact("a")), True),
        ("all-injective-match-required", all_c(exact("a")), all_c(exact("a"), exact("a")), False),
        ("any-removes-branch", any_c(exact("a")), any_c(exact("a"), exact("b")), True),
        ("any-adds-unparented-branch", any_c(exact("a"), exact("x")), any_c(one_of("a", "b"), exact("c")), False),
        ("any-cross-type-covered", any_c(exact("a")), any_c(one_of("a", "b")), True),
        ("all-narrows-wildcard", all_c(exact("a")), wildcard(), True),
        ("any-narrows-wildcard", any_c(exact("a")), wildcard(), True),
        ("exact-cannot-subsume-subset", exact(["a"]), subset("a", "b"), False),
        ("true-versus-one-not-equal", exact(True), exact(1), False),
    ]


def validation_cases() -> list[tuple[str, Any, str]]:
    depth_tree: dict[str, Any] = wildcard()
    for _ in range(FROZEN_MAX_DEPTH):
        depth_tree = all_c(depth_tree)
    broad_tree = all_c(*(wildcard() for _ in range(FROZEN_MAX_CLAUSES + 1)))
    node_tree: dict[str, Any] = wildcard()
    for _ in range(9):
        node_tree = all_c(node_tree, copy.deepcopy(node_tree))
    deep_value: Any = "leaf"
    for _ in range(FROZEN_MAX_CONSTRAINT_VALUE_DEPTH):
        deep_value = [deep_value]
    oversized_value_array = [f"value-{index}" for index in range(FROZEN_MAX_CONSTRAINT_VALUE_NODES)]
    return [
        ("unknown-constraint-extension", {"constraint_type": "regex", "pattern": ".*"}, "constraint-type-unsupported"),
        ("unexpected-exact-member", {"constraint_type": "exact", "value": "a", "ignored": True}, "constraint-member-unsupported"),
        ("exact-object-value", exact({"k": "v"}), "exact-value-invalid"),
        ("exact-nonfinite-value", exact(float("inf")), "exact-value-invalid"),
        ("range-bool-bound", range_c(min=True), "range-bound-invalid"),
        ("range-exclusive-empty", range_c(min=1, max=1, min_inclusive=False), "range-empty"),
        ("any-empty", any_c(), "any-empty"),
        ("constraint-depth-overflow", depth_tree, "constraint-depth-exceeded"),
        ("constraint-clause-overflow", broad_tree, "constraint-clause-limit-exceeded"),
        ("constraint-node-overflow", node_tree, "constraint-node-limit-exceeded"),
        ("constraint-value-depth-overflow", contains(deep_value), "constraint-value-depth-exceeded"),
        ("constraint-value-node-overflow", contains(*oversized_value_array), "constraint-value-node-limit-exceeded"),
    ]


def attenuation_cases() -> list[tuple[str, dict[str, Any], dict[str, Any], bool]]:
    return [
        ("same-tool-narrow-constraint", tools_for(one_of("a", "b")), tools_for(exact("a")), True),
        ("child-adds-tool", {"read_file": {}}, {"read_file": {}, "delete_file": {}}, False),
        ("constrained-argument-key-added", {"read_file": {"path": exact("a")}},
         {"read_file": {"path": exact("a"), "secret": wildcard()}}, False),
        ("constrained-argument-key-dropped", {"read_file": {"path": exact("a"), "mode": exact("ro")}},
         {"read_file": {"path": exact("a")}}, False),
        ("open-world-parent-may-constrain", {"read_file": {}},
         {"read_file": {"path": exact("a")}}, True),
        ("constraint-widened", {"read_file": {"path": one_of("a")}},
         {"read_file": {"path": one_of("a", "b")}}, False),
        ("empty-parent-tools", {}, {}, True),
        ("empty-parent-tool-set-cannot-grow", {}, {"read_file": {}}, False),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.aat-capability-subsumption-differential-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "constraint_controls": [],
        "validation_controls": [],
        "attenuation_controls": [],
        "invocation_controls": [],
        "mutations": [],
    }
    try:
        require(aat.MAX_CONSTRAINT_DEPTH == FROZEN_MAX_DEPTH, "constraint-depth limit drifted")
        require(aat.MAX_CONSTRAINT_NODES == FROZEN_MAX_NODES, "constraint-node limit drifted")
        require(aat.MAX_COMPOSITE_CLAUSES == FROZEN_MAX_CLAUSES, "composite-clause limit drifted")
        require(aat.MAX_TOOLS_PER_TOKEN == FROZEN_MAX_TOOLS, "per-token tool-count limit drifted")
        require(aat.MAX_CONSTRAINTS_PER_TOOL == FROZEN_MAX_ARGUMENT_KEYS,
                "per-tool argument-constraint limit drifted")
        require(aat.MAX_CONSTRAINT_VALUE_DEPTH == FROZEN_MAX_CONSTRAINT_VALUE_DEPTH,
                "nested constraint-value depth limit drifted")
        require(aat.MAX_CONSTRAINT_VALUE_NODES == FROZEN_MAX_CONSTRAINT_VALUE_NODES,
                "nested constraint-value node limit drifted")
        require(aat.MAX_TOOL_NAME_BYTES == FROZEN_MAX_TOOL_NAME_BYTES,
                "tool-name byte limit drifted")
        require(aat.MAX_INVOCATION_VALUE_DEPTH == FROZEN_MAX_INVOCATION_VALUE_DEPTH,
                "invocation JSON-value depth limit drifted")
        require(aat.MAX_INVOCATION_VALUE_NODES == FROZEN_MAX_INVOCATION_VALUE_NODES,
                "invocation JSON-value node limit drifted")
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        module_path = HERE / "aat_capability_subsumption.py"
        receipt["module_sha256"] = hashlib.sha256(module_path.read_bytes()).hexdigest()
        receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()

        for name, derived, parent, expected in subsumption_cases():
            observed = aat.constraint_subsumes(derived, parent)
            require(observed is expected,
                    f"{name}: expected subsumption={expected}, got {observed}")
            if observed:
                require(independently_sound_subsumption(derived, parent),
                        f"{name}: implementation PASS disagrees with finite raw-JSON denotation")
            receipt["constraint_controls"].append({
                "id": name, "expected_subsumes": expected, "observed_subsumes": observed,
                "independent_finite_soundness": "PASS" if observed else "NOT_APPLICABLE",
            })

        for name, constraint, expected_code in validation_cases():
            try:
                aat.validate_constraint(constraint)
            except aat.CapabilityError as error:
                require(error.code == expected_code,
                        f"{name}: expected code {expected_code}, got {error.code}")
                receipt["validation_controls"].append({
                    "id": name, "rejected": True, "finding": error.code,
                })
            else:
                raise AssertionError(f"{name}: malformed or over-limit constraint was accepted")

        # Protocol resource limits are checked at the authorization_details boundary,
        # not only on individual recursive constraint trees.
        too_many_tools = {f"tool-{index}": {} for index in range(FROZEN_MAX_TOOLS + 1)}
        try:
            aat.validate_authorization_details(aat_details(too_many_tools), require_one=True,
                                               label="tool-count-bound")
        except aat.CapabilityError as error:
            require(error.code == "tool-count-limit-exceeded",
                    "tool-count bound returned an unexpected finding: " + error.code)
            receipt["validation_controls"].append({
                "id": "tool-count-limit-exceeded", "rejected": True, "finding": error.code,
            })
        else:
            raise AssertionError("tool-count-limit-exceeded: oversized tools map was accepted")

        too_many_arguments = {
            f"arg-{index}": wildcard() for index in range(FROZEN_MAX_ARGUMENT_KEYS + 1)
        }
        try:
            aat.validate_authorization_details(
                aat_details({"read_file": too_many_arguments}),
                require_one=True,
                label="argument-count-bound",
            )
        except aat.CapabilityError as error:
            require(error.code == "argument-key-limit-exceeded",
                    "argument-count bound returned an unexpected finding: " + error.code)
            receipt["validation_controls"].append({
                "id": "argument-key-limit-exceeded", "rejected": True, "finding": error.code,
            })
        else:
            raise AssertionError("argument-key-limit-exceeded: oversized argument map was accepted")

        oversized_tool_name = "t" * (FROZEN_MAX_TOOL_NAME_BYTES + 1)
        try:
            aat.validate_authorization_details(
                aat_details({oversized_tool_name: {}}),
                require_one=True,
                label="tool-name-bound",
            )
        except aat.CapabilityError as error:
            require(error.code == "tool-name-limit-exceeded",
                    "tool-name length bound returned unexpected finding: " + error.code)
            receipt["validation_controls"].append({
                "id": "tool-name-limit-exceeded", "rejected": True, "finding": error.code,
            })
        else:
            raise AssertionError("tool-name-limit-exceeded: oversized tool name was accepted")

        for name, parent_tools, child_tools, expected in attenuation_cases():
            try:
                aat.check_capability_attenuation(parent_tools, child_tools)
                observed = True
                failure_code = None
            except aat.CapabilityError as error:
                observed = False
                failure_code = error.code
            require(observed is expected,
                    f"{name}: expected attenuation={expected}, got {observed} ({failure_code})")
            receipt["attenuation_controls"].append({
                "id": name, "expected_pass": expected, "observed_pass": observed,
                "finding": failure_code,
            })

        runtime_cases = [
            ("exact-match", exact("a"), "a", True),
            ("exact-reject", exact("a"), "b", False),
            ("range-match", range_c(min=1, max=3), 2, True),
            ("range-reject-boundary", range_c(max=3, max_inclusive=False), 3, False),
            ("one-of-match", one_of("a", "b"), "b", True),
            ("not-one-of-reject", not_one_of("x"), "x", False),
            ("contains-match", contains("a", "b"), ["b", "a", "c"], True),
            ("contains-reject", contains("a", "b"), ["a"], False),
            ("subset-match", subset("a", "b"), ["b"], True),
            ("subset-reject", subset("a"), ["a", "b"], False),
            ("wildcard-match", wildcard(), {"arbitrary": True}, True),
            ("all-match", all_c(one_of(1, 2), range_c(min=1, max=2)), 2, True),
            ("all-reject", all_c(one_of(1, 2), range_c(min=1, max=2)), 3, False),
            ("any-match", any_c(exact("a"), exact("b")), "b", True),
            ("any-reject", any_c(exact("a"), exact("b")), "c", False),
        ]
        for name, constraint, value, expected in runtime_cases:
            observed = aat.constraint_accepts(constraint, value)
            require(observed is expected, f"{name}: runtime predicate mismatch")
            independent = independent_accepts(constraint, value)
            require(observed is independent, f"{name}: runtime result disagrees with independent replay")
            receipt["invocation_controls"].append({
                "id": name, "expected_accept": expected, "observed_accept": observed,
                "independent_replay": "PASS",
            })

        tools = {"read_file": {"path": exact("/public.txt")}}
        invocation_cases = [
            ("allowed-invocation", tools, "read_file", {"path": "/public.txt"}, True),
            ("value-rejected", tools, "read_file", {"path": "/private.txt"}, False),
            ("extra-argument-rejected", tools, "read_file", {"path": "/public.txt", "admin": True}, False),
            ("missing-argument-rejected", tools, "read_file", {}, False),
            ("unknown-tool-rejected", tools, "delete_file", {}, False),
            ("open-world-tool-allows-args", {"search": {}}, "search", {"any": "value"}, True),
        ]
        deep_invocation_value: Any = "leaf"
        for _ in range(FROZEN_MAX_INVOCATION_VALUE_DEPTH):
            deep_invocation_value = [deep_invocation_value]
        invocation_value_nodes = [f"value-{index}" for index in range(FROZEN_MAX_INVOCATION_VALUE_NODES)]
        invocation_cases.extend([
            ("invocation-value-depth-overflow",
             {"search": {}}, "search", {"query": deep_invocation_value}, False),
            ("invocation-value-node-overflow",
             {"search": {}}, "search", {"query": invocation_value_nodes}, False),
        ])
        for name, candidate_tools, tool, values, expected in invocation_cases:
            try:
                aat.validate_invocation(candidate_tools, tool, values)
                observed = True
                finding = None
            except aat.CapabilityError as error:
                observed = False
                finding = error.code
            independent = independently_invocation_check(candidate_tools, tool, values)
            require(observed is expected, f"{name}: invocation expected={expected}, got={observed}")
            require(observed is independent, f"{name}: invocation disagrees with independent raw-JSON replay")
            receipt["invocation_controls"].append({
                "id": name, "expected_accept": expected, "observed_accept": observed,
                "independent_replay": "PASS", "finding": finding,
            })

        # Mutation sensitivity: each omitted core operation should allow a
        # fixture that independent set/shape semantics reject.
        mutation_cases = (
            ("constraint-subsumption-bypassed", "constraint_subsumes",
             lambda *args, **kwargs: True,
             lambda: aat.constraint_subsumes(exact("b"), exact("a")),
             False),
            ("tool-set-attenuation-bypassed", "check_capability_attenuation",
             lambda *args, **kwargs: None,
             lambda: aat.check_capability_attenuation({"read_file": {}}, {"delete_file": {}}),
             False),
            ("runtime-constraint-bypassed", "constraint_accepts",
             lambda *args, **kwargs: True,
             lambda: aat.constraint_accepts(exact("allowed"), "forbidden"),
             False),
            ("invocation-shape-bypassed", "validate_invocation",
             lambda *args, **kwargs: None,
             lambda: aat.validate_invocation({"read_file": {"path": exact("a")}},
                                             "read_file", {"path": "a", "extra": "x"}),
             False),
        )
        for mutation_id, attribute, mutant, probe, safe_expected in mutation_cases:
            original = getattr(aat, attribute)
            try:
                setattr(aat, attribute, mutant)
                try:
                    result = probe()
                    accepted = bool(result) if attribute != "validate_invocation" and attribute != "check_capability_attenuation" else True
                except aat.CapabilityError:
                    accepted = False
            finally:
                setattr(aat, attribute, original)
            require(accepted is not safe_expected,
                    mutation_id + ": the probe did not expose the mutation")
            receipt["mutations"].append({
                "id": mutation_id, "mutant_was_observable": True,
                "known_bad_probe_accepted": accepted,
            })

        receipt["status"] = "PASS"
        receipt["summary"] = {
            "constraint_subsumption_controls": len(receipt["constraint_controls"]),
            "malformed_or_bound_controls": len(receipt["validation_controls"]),
            "capability_attenuation_controls": len(receipt["attenuation_controls"]),
            "runtime_and_invocation_controls": len(receipt["invocation_controls"]),
            "mutants_injected": len(receipt["mutations"]),
            "mutants_detected": sum(bool(row["mutant_was_observable"]) for row in receipt["mutations"]),
            "resource_limits": {
                "max_constraint_depth": FROZEN_MAX_DEPTH,
                "max_constraint_nodes": FROZEN_MAX_NODES,
                "max_composite_clauses": FROZEN_MAX_CLAUSES,
                "max_tools_per_token": FROZEN_MAX_TOOLS,
                "max_constraints_per_tool": FROZEN_MAX_ARGUMENT_KEYS,
                "max_constraint_value_depth": FROZEN_MAX_CONSTRAINT_VALUE_DEPTH,
                "max_constraint_value_nodes": FROZEN_MAX_CONSTRAINT_VALUE_NODES,
                "max_tool_name_bytes": FROZEN_MAX_TOOL_NAME_BYTES,
                "max_invocation_value_depth": FROZEN_MAX_INVOCATION_VALUE_DEPTH,
                "max_invocation_value_nodes": FROZEN_MAX_INVOCATION_VALUE_NODES,
            },
            "bounded_denotation_soundness": "PASS_FOR_RETURNED_SUBSUMPTION_PASSES",
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AAT CAPABILITY SUBSUMPTION PASS: core type-pair and tool/argument attenuation controls")
        print("INDEPENDENT FINITE-DENOTATION SOUNDNESS: checked all reported subsumption PASS results")
        print("CAPABILITY MUTATION SENSITIVITY PASS: 4 of 4 semantic checks proved necessary")
        print("QUALIFICATION NOT CLAIMED: bounded research/specification candidate")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AAT CAPABILITY SUBSUMPTION FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
