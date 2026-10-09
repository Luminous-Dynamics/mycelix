#!/usr/bin/env python3
"""Bounded AAT core capability and argument-constraint semantics.

This module implements the core attenuation relation and invocation predicate
for AAT authorization_details. It is intentionally fail-closed for unknown
constraint types and uses finite implementation limits for constraint depth,
node count, and composite clause fan-out. It is a research candidate, not a
claim of full AAT implementation or production qualification.
"""
from __future__ import annotations

import math
from typing import Any

MAX_CONSTRAINT_DEPTH = 32
MAX_CONSTRAINT_NODES = 512
MAX_COMPOSITE_CLAUSES = 128
MAX_TOOLS_PER_TOKEN = 256
MAX_CONSTRAINTS_PER_TOOL = 64
MAX_CONSTRAINT_VALUE_DEPTH = 32
MAX_CONSTRAINT_VALUE_NODES = 512
MAX_TOOL_NAME_BYTES = 256
MAX_INVOCATION_VALUE_DEPTH = 32
MAX_INVOCATION_VALUE_NODES = 4096

CORE_TYPES = {
    "exact", "range", "one_of", "not_one_of", "contains",
    "subset", "wildcard", "all", "any",
}


class CapabilityError(ValueError):
    def __init__(self, code: str, detail: str, path: str = "$"):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.path = path


def _kind(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, str):
        return "string"
    if isinstance(value, (int, float)):
        return "number"
    if isinstance(value, list):
        return "array"
    if isinstance(value, dict):
        return "object"
    return "unsupported"


def strict_equal(left: Any, right: Any) -> bool:
    """Iterative JSON-semantic equality without bool/int or recursion pitfalls."""
    pending: list[tuple[Any, Any]] = [(left, right)]
    while pending:
        left_value, right_value = pending.pop()
        left_kind, right_kind = _kind(left_value), _kind(right_value)
        if left_kind != right_kind:
            return False
        if left_kind == "object":
            if set(left_value) != set(right_value):
                return False
            pending.extend((left_value[key], right_value[key]) for key in left_value)
            continue
        if left_kind == "array":
            if len(left_value) != len(right_value):
                return False
            pending.extend(zip(left_value, right_value))
            continue
        if left_kind == "number":
            if left_value != right_value:
                return False
            continue
        if left_kind == "unsupported" or left_value != right_value:
            return False
    return True


def _has_nonfinite_number(value: Any) -> bool:
    """Iterative check so nested untrusted values cannot exhaust Python recursion."""
    stack = [value]
    while stack:
        current = stack.pop()
        if type(current) is float and not math.isfinite(current):
            return True
        if isinstance(current, list):
            stack.extend(current)
        elif isinstance(current, dict):
            stack.extend(current.values())
    return False


def _validate_constraint_value_tree(value: Any, *, path: str, constraint_type: str,
                                   max_depth: int = MAX_CONSTRAINT_VALUE_DEPTH,
                                   max_nodes: int = MAX_CONSTRAINT_VALUE_NODES,
                                   error_prefix: str = "constraint") -> None:
    """Bound JSON values embedded in constraints or supplied invocation arguments."""
    stack: list[tuple[Any, int, str]] = [(value, 1, path)]
    nodes = 0
    while stack:
        current, depth, current_path = stack.pop()
        nodes += 1
        if nodes > max_nodes:
            raise CapabilityError(
                f"{error_prefix}-value-node-limit-exceeded",
                f"{error_prefix} JSON values exceed {max_nodes} nodes",
                current_path,
            )
        if depth > max_depth:
            raise CapabilityError(
                f"{error_prefix}-value-depth-exceeded",
                f"{error_prefix} JSON values exceed depth {max_depth}",
                current_path,
            )
        if current is None or type(current) in (bool, int, str):
            continue
        if type(current) is float:
            if not math.isfinite(current):
                raise CapabilityError(
                    f"{constraint_type}-value-nonfinite",
                    f"{constraint_type} values cannot contain non-finite numbers",
                    current_path,
                )
            continue
        if isinstance(current, list):
            for index, child in enumerate(current):
                stack.append((child, depth + 1, f"{current_path}[{index}]"))
            continue
        if isinstance(current, dict):
            for key, child in current.items():
                if not isinstance(key, str):
                    raise CapabilityError(
                        "constraint-value-object-key-invalid",
                        "constraint JSON object keys must be strings",
                        current_path,
                    )
                stack.append((child, depth + 1, f"{current_path}.{key}"))
            continue
        raise CapabilityError(
            "constraint-value-invalid",
            "constraint values must be JSON-compatible",
            current_path,
        )


def _is_scalar(value: Any) -> bool:
    if _kind(value) == "number":
        return type(value) is int or (type(value) is float and math.isfinite(value))
    return _kind(value) in {"null", "boolean", "string"}


def _is_finite_number(value: Any) -> bool:
    if type(value) is int:
        return True
    return type(value) is float and math.isfinite(value)


def _require_exact_members(raw: dict[str, Any], permitted: set[str], path: str) -> None:
    unexpected = sorted(set(raw) - permitted)
    if unexpected:
        raise CapabilityError(
            "constraint-member-unsupported",
            f"unsupported members for constraint at {path}: {unexpected}",
            path,
        )


def validate_constraint(constraint: Any, *, path: str = "$",
                        max_depth: int = MAX_CONSTRAINT_DEPTH,
                        max_nodes: int = MAX_CONSTRAINT_NODES) -> None:
    """Validate one complete constraint tree, including recursive bounds."""
    count = 0

    def visit(node: Any, current_path: str, depth: int) -> None:
        nonlocal count
        count += 1
        if count > max_nodes:
            raise CapabilityError(
                "constraint-node-limit-exceeded",
                f"constraint tree exceeds {max_nodes} nodes",
                current_path,
            )
        if depth > max_depth:
            raise CapabilityError(
                "constraint-depth-exceeded",
                f"constraint tree exceeds depth {max_depth}",
                current_path,
            )
        if not isinstance(node, dict):
            raise CapabilityError("constraint-not-object", "constraint must be an object", current_path)
        ctype = node.get("constraint_type")
        if not isinstance(ctype, str) or ctype not in CORE_TYPES:
            raise CapabilityError(
                "constraint-type-unsupported",
                f"unknown or invalid constraint_type {ctype!r}",
                current_path,
            )

        if ctype == "exact":
            _require_exact_members(node, {"constraint_type", "value"}, current_path)
            if ("value" not in node or not _is_scalar(node["value"])
                    or _has_nonfinite_number(node.get("value"))):
                raise CapabilityError("exact-value-invalid", "exact.value must be a JSON scalar", current_path)
            return

        if ctype == "range":
            _require_exact_members(
                node,
                {"constraint_type", "min", "max", "min_inclusive", "max_inclusive"},
                current_path,
            )
            for bound in ("min", "max"):
                if bound in node and not _is_finite_number(node[bound]):
                    raise CapabilityError(
                        "range-bound-invalid",
                        f"range.{bound} must be a finite JSON number",
                        current_path,
                    )
            for flag in ("min_inclusive", "max_inclusive"):
                if flag in node and type(node[flag]) is not bool:
                    raise CapabilityError(
                        "range-inclusivity-invalid",
                        f"range.{flag} must be a boolean",
                        current_path,
                    )
            lo, hi = node.get("min"), node.get("max")
            if lo is not None and hi is not None:
                if lo > hi:
                    raise CapabilityError("range-bounds-inverted", "range.min exceeds range.max", current_path)
                if lo == hi and (
                    node.get("min_inclusive", True) is False
                    or node.get("max_inclusive", True) is False
                ):
                    raise CapabilityError("range-empty", "equal range bounds cannot exclude an endpoint", current_path)
            return

        if ctype in {"one_of", "not_one_of", "contains", "subset"}:
            member = {
                "one_of": "values",
                "not_one_of": "excluded",
                "contains": "required",
                "subset": "allowed",
            }[ctype]
            _require_exact_members(node, {"constraint_type", member}, current_path)
            if not isinstance(node.get(member), list):
                raise CapabilityError(f"{ctype}-array-invalid", f"{ctype}.{member} must be an array", current_path)
            _validate_constraint_value_tree(
                node[member], path=f"{current_path}.{member}", constraint_type=ctype
            )
            if ctype in {"one_of", "not_one_of"} and any(
                not _is_scalar(value) for value in node[member]
            ):
                raise CapabilityError(
                    f"{ctype}-value-invalid",
                    f"{ctype}.{member} members must be JSON scalars",
                    current_path,
                )
            return

        if ctype == "wildcard":
            _require_exact_members(node, {"constraint_type"}, current_path)
            return

        if ctype in {"all", "any"}:
            _require_exact_members(node, {"constraint_type", "constraints"}, current_path)
            clauses = node.get("constraints")
            if not isinstance(clauses, list):
                raise CapabilityError(
                    f"{ctype}-clauses-invalid",
                    f"{ctype}.constraints must be an array",
                    current_path,
                )
            if len(clauses) > MAX_COMPOSITE_CLAUSES:
                raise CapabilityError(
                    "constraint-clause-limit-exceeded",
                    f"{ctype} has more than {MAX_COMPOSITE_CLAUSES} clauses",
                    current_path,
                )
            if ctype == "any" and not clauses:
                raise CapabilityError("any-empty", "any.constraints must contain at least one clause", current_path)
            for index, child in enumerate(clauses):
                visit(child, f"{current_path}.constraints[{index}]", depth + 1)
            return

        raise CapabilityError("constraint-type-unsupported", f"unsupported constraint type {ctype!r}", current_path)

    visit(constraint, path, 1)


def _contains_member(values: list[Any], wanted: Any) -> bool:
    return any(strict_equal(value, wanted) for value in values)


def _range_contains(constraint: dict[str, Any], value: Any) -> bool:
    if not _is_finite_number(value):
        return False
    lo, hi = constraint.get("min"), constraint.get("max")
    if lo is not None:
        if value < lo or (value == lo and constraint.get("min_inclusive", True) is False):
            return False
    if hi is not None:
        if value > hi or (value == hi and constraint.get("max_inclusive", True) is False):
            return False
    return True


def constraint_accepts(constraint: Any, value: Any) -> bool:
    """Deterministic runtime predicate for a previously or freshly validated tree."""
    validate_constraint(constraint)
    ctype = constraint["constraint_type"]
    if ctype == "exact":
        return strict_equal(value, constraint["value"])
    if ctype == "range":
        return _range_contains(constraint, value)
    if ctype == "one_of":
        return _contains_member(constraint["values"], value)
    if ctype == "not_one_of":
        return not _contains_member(constraint["excluded"], value)
    if ctype == "contains":
        required = constraint["required"]
        return isinstance(value, list) and all(_contains_member(value, item) for item in required)
    if ctype == "subset":
        allowed = constraint["allowed"]
        return isinstance(value, list) and all(_contains_member(allowed, item) for item in value)
    if ctype == "wildcard":
        return True
    if ctype == "all":
        return all(constraint_accepts(child, value) for child in constraint["constraints"])
    if ctype == "any":
        return any(constraint_accepts(child, value) for child in constraint["constraints"])
    return False


def _numeric_value_in_range(value: Any, parent: dict[str, Any]) -> bool:
    return _range_contains(parent, value)


def _bound_min_at_least(derived: dict[str, Any], parent: dict[str, Any]) -> bool:
    parent_min, derived_min = parent.get("min"), derived.get("min")
    if parent_min is None:
        return True
    if derived_min is None or derived_min < parent_min:
        return False
    if derived_min > parent_min:
        return True
    parent_inclusive = parent.get("min_inclusive", True)
    derived_inclusive = derived.get("min_inclusive", True)
    return (not derived_inclusive) or parent_inclusive


def _bound_max_at_most(derived: dict[str, Any], parent: dict[str, Any]) -> bool:
    parent_max, derived_max = parent.get("max"), derived.get("max")
    if parent_max is None:
        return True
    if derived_max is None or derived_max > parent_max:
        return False
    if derived_max < parent_max:
        return True
    parent_inclusive = parent.get("max_inclusive", True)
    derived_inclusive = derived.get("max_inclusive", True)
    return (not derived_inclusive) or parent_inclusive


def constraint_subsumes(derived: Any, parent: Any) -> bool:
    """Return whether the derived constraint denotes a subset of the parent."""
    validate_constraint(derived)
    validate_constraint(parent)
    d_type, p_type = derived["constraint_type"], parent["constraint_type"]

    # Any valid concrete rule narrows the unconstrained wildcard domain.
    if p_type == "wildcard":
        return True

    if d_type == "exact":
        value = derived["value"]
        if p_type == "exact":
            return strict_equal(value, parent["value"])
        if p_type == "range":
            return _numeric_value_in_range(value, parent)
        if p_type == "one_of":
            return _contains_member(parent["values"], value)
        return False

    if d_type == "range":
        if p_type != "range":
            return False
        return _bound_min_at_least(derived, parent) and _bound_max_at_most(derived, parent)

    if d_type == "one_of":
        if p_type != "one_of":
            return False
        return all(_contains_member(parent["values"], value) for value in derived["values"])

    if d_type == "not_one_of":
        if p_type != "not_one_of":
            return False
        return all(_contains_member(derived["excluded"], value) for value in parent["excluded"])

    if d_type == "contains":
        if p_type != "contains":
            return False
        return all(_contains_member(derived["required"], value) for value in parent["required"])

    if d_type == "subset":
        if p_type != "subset":
            return False
        return all(_contains_member(parent["allowed"], value) for value in derived["allowed"])

    if d_type == "wildcard":
        return p_type == "wildcard"

    if d_type == "all":
        if p_type != "all":
            return False
        derived_clauses = derived["constraints"]
        parent_clauses = parent["constraints"]
        if len(derived_clauses) < len(parent_clauses):
            return False
        # Independent bounded bipartite matching; each parent obligation must
        # map to a distinct derived clause that subsumes it.
        candidates = [
            [j for j, d_clause in enumerate(derived_clauses)
             if constraint_subsumes(d_clause, p_clause)]
            for p_clause in parent_clauses
        ]
        matched_derived: dict[int, int] = {}

        def augment(parent_index: int, seen: set[int]) -> bool:
            for derived_index in candidates[parent_index]:
                if derived_index in seen:
                    continue
                seen.add(derived_index)
                prior_parent = matched_derived.get(derived_index)
                if prior_parent is None or augment(prior_parent, seen):
                    matched_derived[derived_index] = parent_index
                    return True
            return False

        for parent_index in range(len(parent_clauses)):
            if not augment(parent_index, set()):
                return False
        return True

    if d_type == "any":
        if p_type != "any":
            return False
        return all(
            any(constraint_subsumes(d_clause, p_clause) for p_clause in parent["constraints"])
            for d_clause in derived["constraints"]
        )

    return False


def validate_authorization_details(details: Any, *, require_one: bool,
                                   label: str = "token") -> dict[str, dict[str, Any]]:
    """Validate AAT details and return the tools map (empty when entry is absent)."""
    if not isinstance(details, list):
        raise CapabilityError("authorization-details-invalid",
                              f"{label}.authorization_details must be an array")
    aat_entries = [
        entry for entry in details if isinstance(entry, dict)
        and entry.get("type") == "attenuating_agent_token"
    ]
    if len(aat_entries) > 1 or (require_one and len(aat_entries) != 1):
        raise CapabilityError(
            "aat-entry-count-invalid",
            f"{label} requires {'exactly one' if require_one else 'at most one'} attenuating_agent_token entry",
        )
    if not aat_entries:
        return {}
    tools = aat_entries[0].get("tools")
    if not isinstance(tools, dict):
        raise CapabilityError("aat-tools-invalid", f"{label} AAT tools must be an object")
    if len(tools) > MAX_TOOLS_PER_TOKEN:
        raise CapabilityError(
            "tool-count-limit-exceeded",
            f"{label} contains more than {MAX_TOOLS_PER_TOKEN} tools",
        )
    for tool_name, arg_constraints in tools.items():
        if not isinstance(tool_name, str) or not tool_name:
            raise CapabilityError("tool-name-invalid", f"{label} tool identifiers must be non-empty strings")
        try:
            tool_name_bytes = tool_name.encode("utf-8")
        except UnicodeEncodeError as error:
            raise CapabilityError("tool-name-invalid",
                                  f"{label} tool identifiers must be valid UTF-8") from error
        if len(tool_name_bytes) > MAX_TOOL_NAME_BYTES:
            raise CapabilityError("tool-name-limit-exceeded",
                                  f"{label} tool identifier exceeds {MAX_TOOL_NAME_BYTES} UTF-8 bytes")
        if not isinstance(arg_constraints, dict):
            raise CapabilityError("tool-constraints-invalid",
                                  f"{label} tool {tool_name!r} constraint map must be an object",
                                  f"$.tools.{tool_name}")
        if len(arg_constraints) > MAX_CONSTRAINTS_PER_TOOL:
            raise CapabilityError("argument-key-limit-exceeded",
                                  f"{label} tool {tool_name!r} exceeds {MAX_CONSTRAINTS_PER_TOOL} argument keys",
                                  f"$.tools.{tool_name}")
        for arg_name, constraint in arg_constraints.items():
            if not isinstance(arg_name, str) or not arg_name:
                raise CapabilityError("argument-name-invalid",
                                      f"{label} argument names must be non-empty strings",
                                      f"$.tools.{tool_name}")
            validate_constraint(constraint, path=f"$.tools.{tool_name}.{arg_name}")
    return tools


def check_capability_attenuation(parent_tools: dict[str, Any],
                                 child_tools: dict[str, Any]) -> None:
    """Raise CapabilityError unless child tools/constraints monotonically narrow parent."""
    if not set(child_tools).issubset(parent_tools):
        added = sorted(set(child_tools) - set(parent_tools))
        raise CapabilityError("tool-capability-expanded",
                              f"derived token adds tool(s) absent from parent: {added}")

    for tool_name, child_constraints in child_tools.items():
        parent_constraints = parent_tools[tool_name]
        if parent_constraints and set(child_constraints) != set(parent_constraints):
            raise CapabilityError(
                "argument-shape-changed",
                f"tool {tool_name!r} changes constrained argument keys under closed-world semantics",
                f"$.tools.{tool_name}",
            )
        for arg_name in set(parent_constraints).intersection(child_constraints):
            if not constraint_subsumes(child_constraints[arg_name], parent_constraints[arg_name]):
                raise CapabilityError(
                    "argument-constraint-expanded",
                    f"derived constraint for {tool_name!r}.{arg_name} does not subsume its parent",
                    f"$.tools.{tool_name}.{arg_name}",
                )


def validate_invocation(tools: dict[str, Any], tool_name: Any, args: Any) -> None:
    """Validate an invocation map with AAT closed-world argument semantics."""
    if not isinstance(tool_name, str) or tool_name not in tools:
        raise CapabilityError("tool-not-authorized", f"tool {tool_name!r} is not authorized")
    if not isinstance(args, dict):
        raise CapabilityError("invocation-args-invalid", "invocation args must be an object")
    _validate_constraint_value_tree(
        args,
        path="$.invocation.args",
        constraint_type="invocation",
        max_depth=MAX_INVOCATION_VALUE_DEPTH,
        max_nodes=MAX_INVOCATION_VALUE_NODES,
        error_prefix="invocation",
    )
    constraints = tools[tool_name]
    if constraints:
        if set(args) != set(constraints):
            raise CapabilityError(
                "invocation-argument-shape-invalid",
                "argument keys must exactly match the non-empty constraint map under closed-world semantics",
            )
        for arg_name, constraint in constraints.items():
            if not constraint_accepts(constraint, args[arg_name]):
                raise CapabilityError(
                    "invocation-constraint-failed",
                    f"argument {arg_name!r} violates its constraint",
                    f"$.tools.{tool_name}.{arg_name}",
                )
