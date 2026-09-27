#!/usr/bin/env python3
"""Oracle-blind conventional SQLite reference adapter for MYC-INT-006F."""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path
from typing import Any

import build_candidate_input as candidate_input

ADAPTER_PROFILE = "myc-int-006f-conventional-sqlite-v1"
RESULT_PROFILE = "oracle-blind-candidate-results-v1"
DISPOSITIONS = frozenset({"Accepted", "Rejected", "Indeterminate", "Unsupported"})


class AdapterError(ValueError):
    pass


def _mapping(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AdapterError(f"{ctx}: expected object")
    return value


def _guard_oracle_blind(value: Any) -> None:
    try:
        candidate_input._walk_forbidden(value)
    except candidate_input.CandidateInputError as exc:
        raise AdapterError(str(exc)) from exc


class ConventionalAdapter:
    def __init__(self, document: dict[str, Any]):
        _guard_oracle_blind(document)
        if document.get("candidate_input_id") != candidate_input.INPUT_ID:
            raise AdapterError("unexpected candidate input identity")
        if document.get("candidate_input_version") != candidate_input.INPUT_VERSION:
            raise AdapterError("unexpected candidate input version")
        if document.get("profile") != candidate_input.INPUT_PROFILE:
            raise AdapterError("unexpected candidate input profile")
        self.document = document
        self.subjects = _mapping(document.get("subjects"), "subjects")
        self.cases = _mapping(document.get("cases"), "cases")
        self.db = sqlite3.connect(":memory:")
        self.db.execute("CREATE TABLE subjects (id TEXT PRIMARY KEY, kind TEXT NOT NULL, namespace TEXT NOT NULL, name TEXT NOT NULL, version TEXT NOT NULL)")
        self.db.execute("CREATE TABLE effects (attempt_id TEXT PRIMARY KEY, executions INTEGER NOT NULL CHECK (executions = 1))")
        self.db.execute("CREATE TABLE decisions (id TEXT PRIMARY KEY, immutable INTEGER NOT NULL CHECK (immutable = 1))")
        self._load_subjects()

    def close(self) -> None:
        self.db.close()

    def _load_subjects(self) -> None:
        for subject_id, raw in sorted(self.subjects.items()):
            subject = _mapping(raw, f"subjects.{subject_id}")
            ref = _mapping(subject.get("semantic_ref"), f"subjects.{subject_id}.semantic_ref")
            self.db.execute(
                "INSERT INTO subjects(id,kind,namespace,name,version) VALUES(?,?,?,?,?)",
                (subject_id, subject.get("kind"), ref.get("namespace"), ref.get("name"), ref.get("version")),
            )
            if subject.get("kind") == "Decision":
                self.db.execute("INSERT INTO decisions(id,immutable) VALUES(?,1)", (subject_id,))
        self.db.commit()

    def kind(self, subject_id: str) -> str:
        row = self.db.execute("SELECT kind FROM subjects WHERE id=?", (subject_id,)).fetchone()
        if row is None:
            raise AdapterError(f"unknown subject: {subject_id}")
        return str(row[0])

    @staticmethod
    def _result(
        case_id: str,
        disposition: str,
        rule: str,
        *,
        effect_count: int = 0,
        tags: list[str] | None = None,
        facts: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if disposition not in DISPOSITIONS:
            raise AdapterError(f"invalid candidate disposition: {disposition}")
        return {
            "case_id": case_id,
            "disposition": disposition,
            "candidate_rule": rule,
            "effect_count": effect_count,
            "tags": sorted(tags or []),
            "facts": dict(sorted((facts or {}).items())),
        }

    def execute_case(self, case_id: str, case: dict[str, Any]) -> dict[str, Any]:
        operation = case.get("operation")
        subjects = case.get("subjects")
        if not isinstance(operation, str) or not isinstance(subjects, list) or not subjects:
            raise AdapterError(f"{case_id}: malformed stimulus")
        kinds = [self.kind(subject) for subject in subjects]

        if operation == "RecordLineage":
            if len(set(subjects)) != len(subjects):
                return self._result(case_id, "Rejected", "lineage-unique-subjects-v1", tags=["duplicate-semantic-subject"], facts={"distinct_semantic_subjects": False})
            return self._result(
                case_id,
                "Accepted",
                "lineage-record-v1",
                tags=["history-preserved"],
                facts={
                    "distinct_semantic_subjects": True,
                    "effect_authority_granted": False if "Recommendation" in kinds else None,
                    "authorization_subject_bound": all(kind in kinds for kind in ("Authorization", "Decision", "Resource")),
                    "historical_mutation": False,
                    "review_candidate_created": all(kind in kinds for kind in ("OutcomeObservation", "ReviewCandidate")),
                },
            )

        if operation == "ExecuteRecommendation":
            return self._result(case_id, "Rejected" if kinds[0] == "Recommendation" else "Unsupported", "authority-kind-gate-v1", tags=["recommendation-no-effect-authority"], facts={"effect_authority_granted": False})

        if operation == "ImportForeignDecisionAsLocalAuthority":
            return self._result(case_id, "Rejected" if kinds[0] == "ForeignDecision" else "Unsupported", "foreign-decision-local-authority-gate-v1", tags=["foreign-decision-unrecognized"], facts={"local_authority_granted": False})

        if operation == "UseCredentialAsAuthority":
            return self._result(case_id, "Rejected" if kinds[0] == "Credential" else "Unsupported", "credential-authority-separation-v1", tags=["credential-not-effect-authority"], facts={"effect_authority_granted": False})

        if operation == "DeliverImplementationAttempt":
            transport = _mapping(case.get("transport"), f"{case_id}.transport")
            if kinds[0] != "ImplementationAttempt":
                return self._result(case_id, "Unsupported", "attempt-idempotency-v1")
            delivery_count = int(transport.get("delivery_count", 0))
            semantic_count = int(transport.get("semantic_attempt_count", 0))
            if delivery_count < 1 or semantic_count != 1:
                return self._result(case_id, "Unsupported", "attempt-idempotency-v1")
            attempt = subjects[0]
            self.db.execute("INSERT OR IGNORE INTO effects(attempt_id,executions) VALUES(?,1)", (attempt,))
            self.db.commit()
            count = self.db.execute("SELECT COUNT(*) FROM effects WHERE attempt_id=?", (attempt,)).fetchone()[0]
            return self._result(case_id, "Accepted", "attempt-idempotency-v1", effect_count=int(count), tags=["single-logical-effect"], facts={"idempotent_replay": True, "logical_effect_count": int(count)})

        if operation == "ClassifyTimeoutOutcome":
            transport = _mapping(case.get("transport"), f"{case_id}.transport")
            if transport.get("receiver_persistence") == "possible" and transport.get("acknowledgement") == "missing" and transport.get("sender_result") == "timeout":
                return self._result(case_id, "Indeterminate", "delivery-knowledge-v1", tags=["unknown-delivery-state"], facts={"unknown_state_preserved": True})
            return self._result(case_id, "Unsupported", "delivery-knowledge-v1")

        if operation == "AdmitStaleSchema":
            schema = _mapping(case.get("schema"), f"{case_id}.schema")
            stale = schema.get("supplied_generation") != schema.get("required_generation")
            return self._result(case_id, "Rejected" if stale else "Accepted", "schema-generation-gate-v1", tags=["schema-generation-mismatch"] if stale else [], facts={"stale_schema_rejected": bool(stale), "source_schema_preserved": True})

        if operation == "AcceptExternalCertificationLocally":
            return self._result(case_id, "Rejected" if kinds[0] == "Certification" else "Unsupported", "external-certification-local-acceptance-v1", tags=["external-certification-unrecognized"], facts={"local_authority_granted": False})

        if operation == "TreatDerivedSummaryAsSourceFact":
            return self._result(case_id, "Rejected" if kinds[0] == "DerivedSummary" else "Unsupported", "source-owned-fact-v1", tags=["derived-view-not-source-state"], facts={"distinct_semantic_subjects": True})

        if operation == "ExecuteWithExpiredAuthorization":
            authority = _mapping(case.get("authority"), f"{case_id}.authority")
            expired = kinds[0] == "Authorization" and authority.get("status") == "expired"
            return self._result(case_id, "Rejected" if expired else "Unsupported", "authorization-validity-v1", tags=["authorization-not-current"] if expired else [], facts={"expired_authority_rejected": bool(expired)})

        if operation == "RewriteHistoricalDecisionFromOutcome":
            if "Decision" in kinds and "OutcomeObservation" in kinds:
                decision_id = subjects[kinds.index("Decision")]
                immutable = self.db.execute("SELECT immutable FROM decisions WHERE id=?", (decision_id,)).fetchone()
                if immutable and immutable[0] == 1:
                    return self._result(case_id, "Rejected", "historical-decision-immutability-v1", tags=["review-not-rewrite"], facts={"historical_mutation": False, "review_candidate_created": "ReviewCandidate" in kinds})
            return self._result(case_id, "Unsupported", "historical-decision-immutability-v1")

        if operation == "ProcessReorderedTransport":
            transport = _mapping(case.get("transport"), f"{case_id}.transport")
            if transport.get("total_order_promised") is False:
                return self._result(case_id, "Accepted", "order-independent-seam-v1", tags=["provenance-preserved"], facts={"provenance_preserved": True, "distinct_semantic_subjects": len(set(subjects)) == len(subjects)})
            return self._result(case_id, "Unsupported", "order-independent-seam-v1")

        if operation == "CollapseSchemaQualifiedIdentity":
            schema = _mapping(case.get("schema"), f"{case_id}.schema")
            mismatch = schema.get("visible_identifier_equal") is True and schema.get("source_schema_equal") is False
            return self._result(case_id, "Rejected" if mismatch else "Unsupported", "schema-qualified-identity-v1", tags=["visible-id-insufficient"] if mismatch else [], facts={"source_schema_preserved": True})

        if operation == "TranslateWithUndeclaredLoss":
            translation = _mapping(case.get("translation"), f"{case_id}.translation")
            undeclared = translation.get("declared_loss") is False and bool(translation.get("actual_loss"))
            return self._result(case_id, "Rejected" if undeclared else "Unsupported", "translation-loss-v1", tags=["loss-must-be-declared"] if undeclared else [], facts={"source_schema_preserved": True, "provenance_preserved": True})

        if operation == "ImportForeignAuthorityAsLocal":
            return self._result(case_id, "Rejected" if kinds[0] == "ForeignAuthority" else "Unsupported", "foreign-authority-firewall-v1", tags=["foreign-authority-unrecognized"], facts={"local_authority_granted": False})

        if operation == "ResolvePartitionByLastArrival":
            transport = _mapping(case.get("transport"), f"{case_id}.transport")
            bad_resolution = transport.get("partitioned") is True and transport.get("reconnected") is True and transport.get("proposed_resolution") == "last_arrival_wins"
            return self._result(case_id, "Rejected" if bad_resolution else "Unsupported", "partition-conflict-v1", tags=["conflict-preserved"] if bad_resolution else [], facts={"conflict_preserved": bool(bad_resolution)})

        if operation == "PromotePredictionToObservation":
            valid_shape = kinds[:2] == ["Prediction", "Observation"]
            return self._result(case_id, "Rejected" if valid_shape else "Unsupported", "epistemic-kind-separation-v1", tags=["prediction-not-observation"] if valid_shape else [], facts={"observation_promotion": False})

        if operation == "PromoteDeliveryReceiptToImplementationReceipt":
            valid_shape = kinds[:2] == ["DeliveryReceipt", "ImplementationReceipt"]
            return self._result(case_id, "Rejected" if valid_shape else "Unsupported", "receipt-role-separation-v1", tags=["delivery-not-implementation"] if valid_shape else [], facts={"receipt_promotion": False})

        if operation == "PromoteExecutionReceiptToDesiredOutcome":
            valid_shape = kinds[:2] == ["ImplementationReceipt", "OutcomeObservation"]
            return self._result(case_id, "Rejected" if valid_shape else "Unsupported", "execution-outcome-separation-v1", tags=["execution-not-outcome"] if valid_shape else [], facts={"outcome_promotion": False})

        return self._result(case_id, "Unsupported", "unsupported-operation-v1")

    def run_all(self) -> dict[str, Any]:
        results = [self.execute_case(case_id, _mapping(case, f"cases.{case_id}")) for case_id, case in sorted(self.cases.items())]
        return {
            "result_profile": RESULT_PROFILE,
            "adapter_profile": ADAPTER_PROFILE,
            "candidate_input_id": self.document["candidate_input_id"],
            "candidate_input_version": self.document["candidate_input_version"],
            "results": results,
        }


def run_document(document: dict[str, Any]) -> dict[str, Any]:
    adapter = ConventionalAdapter(document)
    try:
        return adapter.run_all()
    finally:
        adapter.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate_input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        document = json.loads(args.candidate_input.read_text(encoding="utf-8"))
        result = run_document(document)
    except (OSError, json.JSONDecodeError, AdapterError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
