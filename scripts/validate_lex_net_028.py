#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

MANIFEST_PATH = Path("docs/lex-net/lex_net_028_manifest.json")
EVIDENCE_DOMAIN = "LEX-NET/EVIDENCE/v3"
AUTHORITY_DOMAIN = "LEX-NET/AUTHORITY/v3"
REQUIREMENT_DOMAIN = "LEX-NET/REQUIREMENT/v1"
SCOPE_KEYS = ("subject", "purpose", "resource", "action")


def canon(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def commitment(domain, fields):
    return hashlib.sha256((domain + "\0" + canon(fields)).encode("utf-8")).hexdigest()


def signed_fields(product):
    return {k: v for k, v in product.items() if k != "commitment"}


def verify_commitment(product, domain):
    return (
        isinstance(product, dict)
        and product.get("commitment") == commitment(domain, signed_fields(product))
    )


def evidence(
    kind,
    evidence_id,
    *,
    subject="org:alpha",
    purpose="customs-submit",
    resource="shipment:123",
    action="submit",
    profile_commitment="profile:v1",
    disposition="positive",
    current_until=200,
    independence_group="issuer:alpha",
    predecessors=None,
    extra=None,
):
    fields = {
        "product_universe": "evidence",
        "kind": kind,
        "evidence_id": evidence_id,
        "subject": subject,
        "purpose": purpose,
        "resource": resource,
        "action": action,
        "profile_commitment": profile_commitment,
        "disposition": disposition,
        "current_until": current_until,
        "independence_group": independence_group,
        "predecessors": list(predecessors or []),
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
    }
    if extra:
        fields.update(extra)
    fields["commitment"] = commitment(EVIDENCE_DOMAIN, fields)
    return fields


def atom(
    kind="RecognitionEvidence",
    *,
    subject="org:alpha",
    purpose="customs-submit",
    resource="shipment:123",
    action="submit",
    profile_commitment="profile:v1",
    accepted_disposition="positive",
    requires_current=True,
    independent_count=1,
):
    return {
        "kind": kind,
        "subject": subject,
        "purpose": purpose,
        "resource": resource,
        "action": action,
        "profile_commitment": profile_commitment,
        "accepted_disposition": accepted_disposition,
        "requires_current": requires_current,
        "independent_count": independent_count,
    }


def requirement(atoms=None, *, alternatives=None, alternative_policy_id=None):
    fields = {
        "atoms": list(atoms or []),
        "alternatives": alternatives or [],
        "alternative_policy_id": alternative_policy_id,
    }
    fields["commitment"] = commitment(REQUIREMENT_DOMAIN, fields)
    return fields


def _all_atom_sets(req):
    if req.get("alternatives"):
        return req["alternatives"]
    return [req.get("atoms", [])]


def requirement_scope(req):
    if not isinstance(req, dict) or not verify_commitment(req, REQUIREMENT_DOMAIN):
        return "EmptyRequirementRejected", None
    atom_sets = _all_atom_sets(req)
    if not atom_sets or any(not option for option in atom_sets):
        return "EmptyRequirementRejected", None
    scopes = {
        tuple(a.get(k) for k in SCOPE_KEYS)
        for option in atom_sets
        for a in option
        if isinstance(a, dict)
    }
    if len(scopes) != 1:
        return "RequirementScopeMismatch", None
    scope_tuple = next(iter(scopes))
    if any(v in (None, "") for v in scope_tuple):
        return "RequirementScopeMismatch", None
    return "Satisfied", dict(zip(SCOPE_KEYS, scope_tuple))


def _prepare(products):
    if any(
        isinstance(p, dict) and p.get("product_universe") == "authority"
        for p in products
    ):
        return "AuthorityProductRejected", [], {}

    seen_id = {}
    dedup = {}
    for p in products:
        if (
            not isinstance(p, dict)
            or p.get("product_universe") != "evidence"
            or not verify_commitment(p, EVIDENCE_DOMAIN)
        ):
            continue
        evidence_id = p.get("evidence_id")
        if not evidence_id:
            continue
        previous = seen_id.get(evidence_id)
        if previous is not None and previous != p["commitment"]:
            return "EvidenceIdentityConflict", [], {}
        seen_id[evidence_id] = p["commitment"]
        dedup[p["commitment"]] = p

    values = list(dedup.values())
    return None, values, {p["evidence_id"]: p for p in values}


def _current_with_closure(product, by_id, evaluation_time, seen=None):
    seen = set(seen or ())
    evidence_id = product["evidence_id"]
    if evidence_id in seen:
        return "MissingDependency", set()
    seen.add(evidence_id)

    current_until = product.get("current_until")
    if current_until is None:
        return "MissingDependency", set()
    if evaluation_time > current_until:
        return "StaleDependency", set()

    closure = {product["commitment"]}
    for predecessor_id in product.get("predecessors", []):
        predecessor = by_id.get(predecessor_id)
        if predecessor is None:
            return "MissingDependency", set()
        outcome, predecessor_closure = _current_with_closure(
            predecessor, by_id, evaluation_time, seen
        )
        if outcome != "Satisfied":
            return outcome, set()
        closure |= predecessor_closure

    return "Satisfied", closure


def _satisfy_atoms(atoms, products, evaluation_time):
    if not atoms:
        return {"outcome": "EmptyRequirementRejected", "support_commitments": []}

    prepared, products, by_id = _prepare(products)
    if prepared:
        return {"outcome": prepared, "support_commitments": []}

    selected_support = set()
    for req_atom in atoms:
        kind_candidates = [p for p in products if p["kind"] == req_atom["kind"]]
        if not kind_candidates:
            return {"outcome": "MissingRequiredEvidence", "support_commitments": []}

        scoped = [
            p
            for p in kind_candidates
            if all(p[k] == req_atom[k] for k in SCOPE_KEYS)
        ]
        if not scoped:
            return {"outcome": "ScopeMismatch", "support_commitments": []}

        profiled = [
            p
            for p in scoped
            if p["profile_commitment"] == req_atom["profile_commitment"]
        ]
        if not profiled:
            return {"outcome": "ProfileMismatch", "support_commitments": []}

        dispositions = {p["disposition"] for p in profiled}
        if len(dispositions) > 1:
            return {"outcome": "ConflictIndeterminate", "support_commitments": []}

        accepted = [
            p
            for p in profiled
            if p["disposition"] == req_atom["accepted_disposition"]
        ]
        if not accepted:
            return {"outcome": "DispositionMismatch", "support_commitments": []}

        usable = []
        current_errors = []
        for p in accepted:
            if req_atom.get("requires_current", True):
                current_outcome, closure = _current_with_closure(
                    p, by_id, evaluation_time
                )
                if current_outcome != "Satisfied":
                    current_errors.append(current_outcome)
                    continue
            else:
                closure = {p["commitment"]}
            usable.append((p, closure))

        if not usable:
            if "MissingDependency" in current_errors:
                return {"outcome": "MissingDependency", "support_commitments": []}
            if "StaleDependency" in current_errors:
                return {"outcome": "StaleDependency", "support_commitments": []}
            return {"outcome": "MissingRequiredEvidence", "support_commitments": []}

        independent_needed = int(req_atom.get("independent_count", 1))
        by_group = {}
        for p, closure in sorted(usable, key=lambda pc: pc[0]["commitment"]):
            by_group.setdefault(p["independence_group"], (p, closure))
        if len(by_group) < independent_needed:
            return {"outcome": "IndependenceInsufficient", "support_commitments": []}

        chosen = sorted(
            by_group.values(), key=lambda pc: pc[0]["commitment"]
        )[:independent_needed]
        for _, closure in chosen:
            selected_support |= closure

    return {
        "outcome": "Satisfied",
        "support_commitments": sorted(selected_support),
    }


def satisfies_detail(req, products, evaluation_time):
    scope_outcome, scope = requirement_scope(req)
    if scope_outcome != "Satisfied":
        return {
            "outcome": scope_outcome,
            "scope": scope,
            "selected_alternative_index": None,
            "support_commitments": [],
        }

    if req.get("alternatives"):
        if not req.get("alternative_policy_id"):
            return {
                "outcome": "AlternativePolicyRequired",
                "scope": scope,
                "selected_alternative_index": None,
                "support_commitments": [],
            }
        failures = []
        for index, option in enumerate(req["alternatives"]):
            result = _satisfy_atoms(option, products, evaluation_time)
            if result["outcome"] == "Satisfied":
                return {
                    **result,
                    "scope": scope,
                    "selected_alternative_index": index,
                }
            failures.append(result["outcome"])
        return {
            "outcome": failures[0] if failures else "MissingRequiredEvidence",
            "scope": scope,
            "selected_alternative_index": None,
            "support_commitments": [],
        }

    result = _satisfy_atoms(req.get("atoms", []), products, evaluation_time)
    return {
        **result,
        "scope": scope,
        "selected_alternative_index": None,
    }


def satisfies(req, products, evaluation_time):
    return satisfies_detail(req, products, evaluation_time)["outcome"]


def evaluate(req, products, evaluation_time, evaluation_id="eval:1"):
    detail = satisfies_detail(req, products, evaluation_time)
    prepared, visible_products, by_id = _prepare(products)
    visible_commitments = (
        sorted(p["commitment"] for p in visible_products)
        if prepared is None
        else []
    )

    support_commitments = detail.get("support_commitments", [])
    support_products = {
        p["commitment"]: p
        for p in visible_products
        if p["commitment"] in support_commitments
    }
    horizons = [
        p["current_until"]
        for p in support_products.values()
        if p.get("current_until") is not None
    ]
    support_current_until = min(horizons) if horizons else None

    scope = detail.get("scope") or {k: "" for k in SCOPE_KEYS}
    return evidence(
        "SatisfactionEvaluation",
        evaluation_id,
        subject=scope["subject"],
        purpose=scope["purpose"],
        resource=scope["resource"],
        action=scope["action"],
        profile_commitment=req.get("commitment", ""),
        disposition=detail["outcome"],
        current_until=support_current_until,
        independence_group="local-evaluator",
        extra={
            "requirement": req,
            "requirement_commitment": req.get("commitment"),
            "visible_evidence_commitments": visible_commitments,
            "support_commitments": support_commitments,
            "support_current_until": support_current_until,
            "evaluation_time": evaluation_time,
            "evaluation_outcome": detail["outcome"],
            "selected_alternative_index": detail.get("selected_alternative_index"),
        },
    )


def local_authorization(
    authorization_id,
    *,
    evaluation_commitment,
    local_domain="domain:a",
    local_grant_commitment="grant:1",
    subject="org:alpha",
    purpose="customs-submit",
    resource="shipment:123",
    action="submit",
    current_until=200,
    max_use_budget=1,
    authorized_operation="mint-capability",
):
    fields = {
        "product_universe": "authority",
        "kind": "LocalAuthorization",
        "authorization_id": authorization_id,
        "evaluation_commitment": evaluation_commitment,
        "local_domain": local_domain,
        "local_grant_commitment": local_grant_commitment,
        "authorized_operation": authorized_operation,
        "subject": subject,
        "purpose": purpose,
        "resource": resource,
        "action": action,
        "current_until": current_until,
        "max_use_budget": max_use_budget,
    }
    fields["commitment"] = commitment(AUTHORITY_DOMAIN, fields)
    return fields


def _evaluation_ok(evaluation):
    return (
        isinstance(evaluation, dict)
        and evaluation.get("product_universe") == "evidence"
        and evaluation.get("kind") == "SatisfactionEvaluation"
        and verify_commitment(evaluation, EVIDENCE_DOMAIN)
        and isinstance(evaluation.get("requirement"), dict)
        and evaluation.get("requirement_commitment")
        == evaluation["requirement"].get("commitment")
        and verify_commitment(evaluation["requirement"], REQUIREMENT_DOMAIN)
        and requirement_scope(evaluation["requirement"])[0] == "Satisfied"
    )


def _authorization_ok(authorization):
    return (
        isinstance(authorization, dict)
        and authorization.get("product_universe") == "authority"
        and authorization.get("kind") == "LocalAuthorization"
        and verify_commitment(authorization, AUTHORITY_DOMAIN)
    )


def mint_local_capability(
    evaluation,
    mint_authorization,
    *,
    mint_time,
    requested_current_until,
    requested_use_budget,
):
    if not _evaluation_ok(evaluation):
        return {"outcome": "EvaluationBindingMismatch"}
    if evaluation.get("evaluation_outcome") != "Satisfied":
        return {"outcome": "MintPreconditionUnsatisfied"}

    support_horizon = evaluation.get("support_current_until")
    evaluation_time = evaluation.get("evaluation_time")
    if (
        support_horizon is None
        or mint_time < evaluation_time
        or mint_time > support_horizon
    ):
        return {"outcome": "EvaluationNotCurrent"}

    if not _authorization_ok(mint_authorization):
        return {"outcome": "MintAuthorizationMismatch"}
    if mint_authorization.get("authorized_operation") != "mint-capability":
        return {"outcome": "MintAuthorizationMismatch"}
    if mint_authorization.get("evaluation_commitment") != evaluation["commitment"]:
        return {"outcome": "MintAuthorizationMismatch"}
    if (
        mint_authorization.get("current_until") is None
        or mint_time > mint_authorization["current_until"]
    ):
        return {"outcome": "MintAuthorizationMismatch"}

    scope = {k: evaluation[k] for k in SCOPE_KEYS}
    if any(mint_authorization.get(k) != value for k, value in scope.items()):
        return {"outcome": "MintAuthorizationMismatch"}

    if (
        not mint_authorization.get("local_domain")
        or not mint_authorization.get("local_grant_commitment")
        or requested_use_budget <= 0
        or requested_use_budget > int(mint_authorization.get("max_use_budget", 0))
    ):
        return {"outcome": "MintAuthorizationMismatch"}

    maximum_end = min(support_horizon, mint_authorization["current_until"])
    if (
        requested_current_until < mint_time
        or requested_current_until > maximum_end
    ):
        return {"outcome": "LeaseLifetimeExceedsSupport"}

    fields = {
        "product_universe": "authority",
        "kind": "LocalCapabilityLease",
        "local_domain": mint_authorization["local_domain"],
        "local_grant_commitment": mint_authorization["local_grant_commitment"],
        **scope,
        "mint_time": mint_time,
        "current_until": requested_current_until,
        "use_budget": requested_use_budget,
        "support_current_until": support_horizon,
        "satisfaction_evaluation_commitment": evaluation["commitment"],
        "mint_authorization_commitment": mint_authorization["commitment"],
    }
    fields["commitment"] = commitment(AUTHORITY_DOMAIN, fields)
    return {"outcome": "Satisfied", "product": fields}


def export_authority_as_evidence(authority, evidence_id):
    return evidence(
        "EvidenceAboutForeignAuthority",
        evidence_id,
        subject=authority["subject"],
        purpose=authority["purpose"],
        resource=authority["resource"],
        action=authority["action"],
        profile_commitment="foreign-authority-evidence:v3",
        disposition="observed",
        current_until=authority["current_until"],
        independence_group="source-domain:" + authority["local_domain"],
        extra={"source_authority_commitment": authority["commitment"]},
    )


def consume_authority(
    authority,
    prior_execution_evidence,
    *,
    execution_id,
    execution_time,
):
    if (
        not isinstance(authority, dict)
        or authority.get("kind") != "LocalCapabilityLease"
        or not verify_commitment(authority, AUTHORITY_DOMAIN)
    ):
        return {"outcome": "AuthorityConsumed"}

    if (
        execution_time < authority["mint_time"]
        or execution_time > authority["current_until"]
    ):
        return {"outcome": "AuthorityNotCurrent"}

    unique = {}
    seen_ids = {}
    for product in prior_execution_evidence:
        if (
            not isinstance(product, dict)
            or product.get("kind") != "ExecutionEvidence"
            or product.get("authority_commitment") != authority["commitment"]
            or not verify_commitment(product, EVIDENCE_DOMAIN)
        ):
            continue
        eid = product.get("evidence_id")
        old = seen_ids.get(eid)
        if old is not None and old != product["commitment"]:
            return {"outcome": "EvidenceIdentityConflict"}
        seen_ids[eid] = product["commitment"]
        unique[product["commitment"]] = product

    if len(unique) >= authority.get("use_budget", 0):
        return {"outcome": "AuthorityConsumed"}

    execution = evidence(
        "ExecutionEvidence",
        execution_id,
        subject=authority["subject"],
        purpose=authority["purpose"],
        resource=authority["resource"],
        action=authority["action"],
        profile_commitment="execution:v3",
        disposition="attempted",
        current_until=authority["current_until"],
        independence_group="executor:" + authority["local_domain"],
        extra={
            "authority_commitment": authority["commitment"],
            "execution_time": execution_time,
            "consumption_index": len(unique) + 1,
        },
    )
    return {
        "outcome": "Satisfied",
        "evidence": execution,
        "authority": authority,
    }


def base_products():
    interpretation = evidence(
        "InterpretationEvidence",
        "i1",
        profile_commitment="interp:v1",
        current_until=150,
        independence_group="parser:a",
    )
    recognition = evidence(
        "RecognitionEvidence",
        "r1",
        profile_commitment="profile:v1",
        current_until=180,
        independence_group="recognizer:a",
        predecessors=["i1"],
    )
    return [interpretation, recognition]


def mint_ready(
    *,
    authorization_scope=None,
    authorization_evaluation=None,
    authorization_until=180,
    max_use_budget=2,
):
    req = requirement([atom()])
    products = base_products()
    evaluation = evaluate(req, products, 100)
    scope = authorization_scope or {
        "subject": "org:alpha",
        "purpose": "customs-submit",
        "resource": "shipment:123",
        "action": "submit",
    }
    authorization = local_authorization(
        "auth:1",
        evaluation_commitment=authorization_evaluation
        or evaluation["commitment"],
        current_until=authorization_until,
        max_use_budget=max_use_budget,
        **scope,
    )
    return req, products, evaluation, authorization


def run_fixture(fixture_id):
    req = requirement([atom()])
    products = base_products()
    now = 100

    if fixture_id == "exact_match":
        return satisfies(req, products, now)
    if fixture_id == "wrong_kind":
        return satisfies(requirement([atom(kind="TranslationEvidence")]), products, now)
    if fixture_id == "scope_mismatch":
        return satisfies(requirement([atom(subject="org:beta")]), products, now)
    if fixture_id == "profile_mismatch":
        return satisfies(
            requirement([atom(profile_commitment="profile:v2")]), products, now
        )
    if fixture_id == "disposition_mismatch":
        return satisfies(
            requirement([atom(accepted_disposition="negative")]), products, now
        )
    if fixture_id == "stale_direct":
        return satisfies(
            req,
            [
                evidence(
                    "RecognitionEvidence",
                    "r2",
                    current_until=50,
                    independence_group="recognizer:a",
                )
            ],
            now,
        )
    if fixture_id == "missing_predecessor":
        return satisfies(
            req,
            [
                evidence(
                    "RecognitionEvidence",
                    "r2",
                    predecessors=["missing"],
                    independence_group="recognizer:a",
                )
            ],
            now,
        )
    if fixture_id == "stale_predecessor":
        dep = evidence(
            "InterpretationEvidence",
            "i2",
            current_until=50,
            profile_commitment="interp:v1",
        )
        product = evidence(
            "RecognitionEvidence",
            "r2",
            predecessors=["i2"],
            independence_group="recognizer:a",
        )
        return satisfies(req, [dep, product], now)
    if fixture_id == "fresh_candidate_survives_stale_extra":
        fresh = evidence(
            "RecognitionEvidence",
            "good",
            current_until=200,
            independence_group="group:a",
        )
        stale = evidence(
            "RecognitionEvidence",
            "old",
            current_until=50,
            independence_group="group:a",
        )
        return satisfies(req, [fresh, stale], now)
    if fixture_id == "duplicate_not_independent":
        p = evidence("RecognitionEvidence", "r2", independence_group="same")
        return satisfies(requirement([atom(independent_count=2)]), [p, dict(p)], now)
    if fixture_id == "correlated_not_independent":
        a = evidence("RecognitionEvidence", "ra", independence_group="same")
        b = evidence("RecognitionEvidence", "rb", independence_group="same")
        return satisfies(requirement([atom(independent_count=2)]), [a, b], now)
    if fixture_id == "independent_two":
        a = evidence("RecognitionEvidence", "ra", independence_group="a")
        b = evidence("RecognitionEvidence", "rb", independence_group="b")
        return satisfies(requirement([atom(independent_count=2)]), [a, b], now)
    if fixture_id == "conflict":
        return satisfies(
            req,
            [
                evidence("RecognitionEvidence", "ra", disposition="positive"),
                evidence("RecognitionEvidence", "rb", disposition="negative"),
            ],
            now,
        )
    if fixture_id == "majority_does_not_override_conflict":
        return satisfies(
            req,
            [
                evidence(
                    "RecognitionEvidence",
                    "ra",
                    disposition="positive",
                    independence_group="a",
                ),
                evidence(
                    "RecognitionEvidence",
                    "rb",
                    disposition="positive",
                    independence_group="b",
                ),
                evidence(
                    "RecognitionEvidence",
                    "rc",
                    disposition="negative",
                    independence_group="c",
                ),
            ],
            now,
        )
    if fixture_id == "alternatives_require_policy":
        return satisfies(
            requirement(
                alternatives=[[atom()], [atom(kind="ExternalProofEvidence")]]
            ),
            products,
            now,
        )
    if fixture_id == "explicit_alternative":
        return satisfies(
            requirement(
                alternatives=[[atom(kind="ExternalProofEvidence")], [atom()]],
                alternative_policy_id="local:v1",
            ),
            products,
            now,
        )
    if fixture_id == "alternative_selected_index_bound":
        alternative_req = requirement(
            alternatives=[[atom(kind="ExternalProofEvidence")], [atom()]],
            alternative_policy_id="local:v1",
        )
        evaluation = evaluate(alternative_req, products, now)
        return (
            "Satisfied"
            if evaluation["evaluation_outcome"] == "Satisfied"
            and evaluation["selected_alternative_index"] == 1
            and evaluation["resource"] == "shipment:123"
            else "EvaluationBindingMismatch"
        )
    if fixture_id == "alternative_scope_mismatch":
        return satisfies(
            requirement(
                alternatives=[
                    [atom(resource="shipment:123")],
                    [atom(resource="shipment:999")],
                ],
                alternative_policy_id="local:v1",
            ),
            products,
            now,
        )
    if fixture_id == "empty_requirement":
        return satisfies(requirement([]), products, now)
    if fixture_id == "authority_rejected_as_evidence":
        _, _, evaluation, authorization = mint_ready()
        minted = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )
        return satisfies(req, [minted["product"]], now)
    if fixture_id == "evidence_id_conflict":
        return satisfies(
            req,
            [
                evidence("RecognitionEvidence", "same"),
                evidence(
                    "RecognitionEvidence", "same", resource="shipment:999"
                ),
            ],
            now,
        )
    if fixture_id == "evaluation_is_bound":
        evaluation = evaluate(req, products, now)
        return (
            "Satisfied"
            if evaluation["kind"] == "SatisfactionEvaluation"
            and evaluation["evaluation_outcome"] == "Satisfied"
            and verify_commitment(evaluation, EVIDENCE_DOMAIN)
            and evaluation["requirement_commitment"] == req["commitment"]
            else "EvaluationBindingMismatch"
        )
    if fixture_id == "evaluation_support_horizon":
        evaluation = evaluate(req, products, now)
        return (
            "Satisfied"
            if evaluation["support_current_until"] == 150
            and len(evaluation["support_commitments"]) == 2
            else "EvaluationBindingMismatch"
        )
    if fixture_id == "bare_string_cannot_mint":
        _, _, _, authorization = mint_ready()
        return mint_local_capability(
            "Satisfied",
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "forged_evaluation_commitment":
        _, _, evaluation, authorization = mint_ready()
        forged = dict(evaluation)
        forged["evaluation_time"] = 101
        return mint_local_capability(
            forged,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "unsatisfied_evaluation_cannot_mint":
        bad_req = requirement([atom(subject="org:beta")])
        evaluation = evaluate(bad_req, base_products(), now)
        authorization = local_authorization(
            "auth:bad",
            evaluation_commitment=evaluation["commitment"],
            subject="org:beta",
        )
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "evidence_cannot_be_mint_authorization":
        _, _, evaluation, _ = mint_ready()
        return mint_local_capability(
            evaluation,
            evidence("RecognitionEvidence", "fake"),
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "mint_authorization_wrong_scope":
        _, _, evaluation, _ = mint_ready()
        authorization = local_authorization(
            "auth:wrong",
            evaluation_commitment=evaluation["commitment"],
            resource="shipment:999",
        )
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "mint_authorization_wrong_evaluation":
        _, _, evaluation, _ = mint_ready()
        authorization = local_authorization(
            "auth:wrong-eval",
            evaluation_commitment="not-this-evaluation",
        )
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "expired_mint_authorization":
        _, _, evaluation, _ = mint_ready()
        authorization = local_authorization(
            "auth:expired",
            evaluation_commitment=evaluation["commitment"],
            current_until=105,
        )
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "delayed_mint_within_support":
        _, _, evaluation, authorization = mint_ready()
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=140,
            requested_current_until=150,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "mint_after_support_expiry":
        _, _, evaluation, authorization = mint_ready()
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=151,
            requested_current_until=151,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "lease_beyond_support_horizon":
        _, _, evaluation, authorization = mint_ready()
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=151,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "lease_beyond_authorization_horizon":
        _, _, evaluation, _ = mint_ready()
        authorization = local_authorization(
            "auth:short",
            evaluation_commitment=evaluation["commitment"],
            current_until=130,
        )
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=131,
            requested_use_budget=1,
        )["outcome"]
    if fixture_id == "use_budget_exceeds_authorization":
        _, _, evaluation, authorization = mint_ready(max_use_budget=1)
        return mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=2,
        )["outcome"]
    if fixture_id == "authorized_mint_binds_both":
        _, _, evaluation, authorization = mint_ready()
        minted = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )
        product = minted.get("product", {})
        return (
            "Satisfied"
            if minted["outcome"] == "Satisfied"
            and product.get("satisfaction_evaluation_commitment")
            == evaluation["commitment"]
            and product.get("mint_authorization_commitment")
            == authorization["commitment"]
            and product.get("support_current_until") == 150
            else "EvaluationBindingMismatch"
        )
    if fixture_id == "export_authority_degrades_to_evidence":
        _, _, evaluation, authorization = mint_ready()
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["product"]
        exported = export_authority_as_evidence(product, "fa")
        return (
            "Satisfied"
            if exported["product_universe"] == "evidence"
            and exported["commitment"] != product["commitment"]
            and not exported["grants_local_authority"]
            else "AuthorityProductRejected"
        )
    if fixture_id == "consume_once_append_only":
        _, _, evaluation, authorization = mint_ready()
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["product"]
        before = product["commitment"]
        consumed = consume_authority(
            product, [], execution_id="x1", execution_time=120
        )
        return (
            "Satisfied"
            if consumed["outcome"] == "Satisfied"
            and consumed["authority"]["commitment"] == before
            and consumed["evidence"]["authority_commitment"] == before
            else "AuthorityConsumed"
        )
    if fixture_id == "consume_twice_rejected":
        _, _, evaluation, authorization = mint_ready()
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=1,
        )["product"]
        first = consume_authority(
            product, [], execution_id="x1", execution_time=120
        )
        return consume_authority(
            product,
            [first["evidence"]],
            execution_id="x2",
            execution_time=121,
        )["outcome"]
    if fixture_id == "duplicate_execution_does_not_double_count":
        _, _, evaluation, authorization = mint_ready(max_use_budget=2)
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=2,
        )["product"]
        first = consume_authority(
            product, [], execution_id="x1", execution_time=120
        )
        return consume_authority(
            product,
            [first["evidence"], dict(first["evidence"])],
            execution_id="x2",
            execution_time=121,
        )["outcome"]
    if fixture_id == "duplicate_execution_id_conflict":
        _, _, evaluation, authorization = mint_ready(max_use_budget=3)
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=140,
            requested_use_budget=3,
        )["product"]
        first = consume_authority(
            product, [], execution_id="x1", execution_time=120
        )["evidence"]
        conflicting = dict(first)
        conflicting["execution_time"] = 121
        conflicting["commitment"] = commitment(
            EVIDENCE_DOMAIN, signed_fields(conflicting)
        )
        return consume_authority(
            product,
            [first, conflicting],
            execution_id="x2",
            execution_time=122,
        )["outcome"]
    if fixture_id == "execution_after_lease_expiry":
        _, _, evaluation, authorization = mint_ready()
        product = mint_local_capability(
            evaluation,
            authorization,
            mint_time=110,
            requested_current_until=130,
            requested_use_budget=1,
        )["product"]
        return consume_authority(
            product, [], execution_id="x1", execution_time=131
        )["outcome"]
    if fixture_id == "execution_evidence_not_recognition":
        return satisfies(
            req,
            [
                evidence(
                    "ExecutionEvidence",
                    "x",
                    profile_commitment="execution:v3",
                )
            ],
            now,
        )
    if fixture_id == "order_invariant":
        a = evidence("RecognitionEvidence", "a", independence_group="a")
        b = evidence("RecognitionEvidence", "b", independence_group="b")
        r = requirement([atom(independent_count=2)])
        return (
            "Satisfied"
            if satisfies(r, [a, b], now)
            == satisfies(r, [b, a], now)
            == "Satisfied"
            else "ConflictIndeterminate"
        )
    if fixture_id == "external_proof_not_recognition":
        return satisfies(
            req,
            [
                evidence(
                    "ExternalProofEvidence",
                    "x",
                    profile_commitment="profile:v1",
                )
            ],
            now,
        )
    raise KeyError(fixture_id)


def load_manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def self_test():
    assert commitment(EVIDENCE_DOMAIN, {"x": 1}) != commitment(
        AUTHORITY_DOMAIN, {"x": 1}
    )
    assert commitment(EVIDENCE_DOMAIN, {"x": 1}) != commitment(
        REQUIREMENT_DOMAIN, {"x": 1}
    )
    manifest = load_manifest()
    assert set(manifest["evidence_kinds"]).isdisjoint(manifest["authority_kinds"])
    req = requirement([atom()])
    assert verify_commitment(req, REQUIREMENT_DOMAIN)
    assert requirement_scope(req)[0] == "Satisfied"
    print("LEX-NET-028 R3 self-test PASS")


def semantic():
    manifest = load_manifest()
    observed = set()
    for fixture in manifest["fixtures"]:
        got = run_fixture(fixture["id"])
        if got != fixture["expected"]:
            raise SystemExit(
                f"fixture {fixture['id']} expected {fixture['expected']} got {got}"
            )
        observed.add(got)
    print(
        json.dumps(
            {
                "tranche": "LEX-NET-028",
                "candidate_revision": "R3",
                "fixture_count": len(manifest["fixtures"]),
                "observed_outcomes": sorted(observed),
                "grants_local_authority": False,
                "grants_external_effect_authority": False,
                "semantic_result": "PASS",
            },
            sort_keys=True,
        )
    )


def scope():
    manifest = load_manifest()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    parent = subprocess.check_output(
        ["git", "rev-parse", "HEAD^"], text=True
    ).strip()
    count = int(
        subprocess.check_output(
            ["git", "rev-list", "--count", f"{parent}..{head}"], text=True
        ).strip()
    )
    paths = sorted(
        subprocess.check_output(
            ["git", "diff", "--name-only", parent, head], text=True
        ).splitlines()
    )
    if parent != manifest["qualified_parent"]:
        raise SystemExit(f"parent mismatch {parent}")
    if count != 1:
        raise SystemExit(f"commit count mismatch {count}")
    if paths != sorted(manifest["expected_paths"]):
        raise SystemExit(f"path set mismatch {paths}")
    print(
        json.dumps(
            {
                "tranche": "LEX-NET-028",
                "candidate_revision": "R3",
                "head": head,
                "parent": parent,
                "commit_count": count,
                "paths": paths,
                "scope_result": "PASS",
            },
            sort_keys=True,
        )
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--semantic", action="store_true")
    parser.add_argument("--scope", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.semantic:
        semantic()
    elif args.scope:
        scope()
    else:
        parser.error("select mode")


if __name__ == "__main__":
    main()
