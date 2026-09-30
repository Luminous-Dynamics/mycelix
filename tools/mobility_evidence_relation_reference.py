#!/usr/bin/env python3
"""Independent semantic validator for Mobility Evidence Relationship Algebra V1."""

RELATIONS = {
    "derived_from": {
        ("design_artifact", "requirement"),
        ("evidence_record", "design_artifact"),
        ("change_set", "evidence_record"),
    },
    "manufactured_as": {("manufacturing_event", "physical_artifact")},
    "inspected_as": {("inspection_record", "physical_artifact")},
    "tested_as": {("test_record", "physical_artifact")},
    "observed_as": {("operational_observation", "physical_artifact")},
    "interprets": {
        ("evidence_record", "inspection_record"),
        ("evidence_record", "test_record"),
        ("evidence_record", "operational_observation"),
    },
    "supersedes": {
        ("design_artifact", "design_artifact"),
        ("evidence_record", "evidence_record"),
        ("change_set", "change_set"),
    },
    "changes": {
        ("change_set", "design_artifact"),
        ("change_set", "physical_artifact"),
        ("change_set", "evidence_record"),
    },
    "requires_revalidation": {
        ("impact_assessment", "revalidation_obligation"),
        ("change_set", "revalidation_obligation"),
    },
    "disputes": {("evidence_record", "evidence_record")},
    "authorizes": {
        ("external_authority_reference", "evidence_record"),
        ("external_authority_reference", "physical_artifact"),
    },
}

NON_IMPLICATIONS = {
    ("derived_from", "manufactured_as"),
    ("interprets", "observed_as"),
    ("changes", "revalidation_complete"),
    ("disputes", "physical_falsehood"),
    ("authorizes", "commons_consensus_authority"),
}


def validate(relation: str, source: str, target: str) -> None:
    if relation not in RELATIONS:
        raise ValueError(f"unknown relation: {relation}")
    if (source, target) not in RELATIONS[relation]:
        raise ValueError(f"invalid endpoint kinds for {relation}: {source}->{target}")


def main() -> None:
    validate("manufactured_as", "manufacturing_event", "physical_artifact")
    validate("interprets", "evidence_record", "operational_observation")
    validate("authorizes", "external_authority_reference", "evidence_record")

    for relation, source, target in [
        ("derived_from", "manufacturing_event", "physical_artifact"),
        ("interprets", "evidence_record", "physical_artifact"),
        ("authorizes", "evidence_record", "evidence_record"),
    ]:
        try:
            validate(relation, source, target)
        except ValueError:
            pass
        else:
            raise AssertionError("semantic substitution was accepted")

    assert ("derived_from", "manufactured_as") in NON_IMPLICATIONS
    assert ("interprets", "observed_as") in NON_IMPLICATIONS
    print("evidence-relationship reference corpus: PASS")


if __name__ == "__main__":
    main()
