#!/usr/bin/env python3
"""Independent semantic reference evaluator for mobility identity/lineage algebra."""
IDENTITY_KINDS = {"requirement","design_revision","configuration_revision","component_instance","manufacturing_event","physical_artifact","inspection_record","test_record","operational_observation","maintenance_event","change_set","evidence_record"}
RELATIONS = {
    "instantiates": {("physical_artifact","design_revision"),("component_instance","design_revision")},
    "configures": {("configuration_revision","design_revision"),("configuration_revision","component_instance")},
    "component_of": {("component_instance","physical_artifact"),("physical_artifact","physical_artifact")},
    "manufactured_from": {("manufacturing_event","design_revision"),("manufacturing_event","configuration_revision")},
    "inspected_as": {("inspection_record","physical_artifact")},
    "tested_as": {("test_record","physical_artifact")},
    "observed_as": {("operational_observation","physical_artifact")},
    "maintained_as": {("maintenance_event","physical_artifact")},
    "repaired_as": {("maintenance_event","physical_artifact")},
    "replaced_by": {("component_instance","component_instance"),("physical_artifact","physical_artifact")},
    "supersedes": {("design_revision","design_revision"),("configuration_revision","configuration_revision"),("evidence_record","evidence_record")},
}
def validate_identity(kind, namespace, identifier):
    return kind in IDENTITY_KINDS and bool(namespace.strip()) and bool(identifier.strip()) and namespace != "holochain" and not identifier.startswith(("uhC0","uhCE"))
def validate_edge(relation, source_kind, target_kind):
    return relation in RELATIONS and (source_kind, target_kind) in RELATIONS[relation]
def self_test():
    assert validate_identity("physical_artifact","mobility","artifact-a")
    assert not validate_identity("physical_artifact","","artifact-a")
    assert not validate_identity("physical_artifact","holochain","uhCkk-example")
    assert validate_edge("instantiates","physical_artifact","design_revision")
    assert validate_edge("replaced_by","component_instance","component_instance")
    assert validate_edge("supersedes","configuration_revision","configuration_revision")
    assert not validate_edge("instantiates","configuration_revision","physical_artifact")
    assert not validate_edge("observed_as","evidence_record","physical_artifact")
if __name__ == "__main__":
    self_test()
    print("mobility identity/lineage reference qualification: PASS")
