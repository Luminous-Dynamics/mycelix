
    assert "id-token: write" not in verifier
    assert "attestations: write" not in verifier
    assert "uses: actions/attest@" not in verifier
    assert "gh attestation verify" not in verifier
    for trigger_key in (
        "D6U_TRIGGER_REPOSITORY:",
        "D6U_TRIGGER_HEAD_BRANCH:",
        "D6U_TRIGGER_HEAD_SHA:",
        "D6U_TRIGGER_RUN_ID:",
        "D6U_TRIGGER_RUN_ATTEMPT:",
    ):
        assert verifier.count(trigger_key) == 1

    assert "id-token: write" in signer
    assert "actions: read" not in signer
    assert "uses: actions/download-artifact@" not in signer
    assert "subject-checksums: ${{ steps.subject_manifest.outputs.manifest }}" in signer
    assert "predicate-path: ${{ steps.commitment_predicate.outputs.predicate }}" in signer
    assert "create-storage-record: false" in signer
    assert "${{ needs.verifier.outputs.canonical_predicate_sha256 }}" in signer
    assert "d6u-attestation-commitment.json" in signer
    assert "attestations: write" in signer
    assert "uses: actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2" in signer
    assert "branches:\n      - myc-int-demo-d6u-holochain-07-runtime" in workflow
    assert workflow.count("github.event.workflow_run.head_branch == 'myc-int-demo-d6u-holochain-07-runtime'") == 3
    assert workflow.count("github.event.workflow_run.repository.full_name == github.repository") == 3
    assert workflow.count("github.event.workflow_run.head_repository.full_name == github.repository") == 3
    assert workflow.count("github.event.workflow_run.name == 'D6U Exact-Head Runtime Executor'") == 3
    assert workflow.count("github.event.workflow_run.path == '.github/workflows/d6u-exact-head-runtime-executor.yml'") == 3
    assert "gh attestation verify" not in signer
    assert "uses: actions/download-artifact@" not in signer

    assert "id-token: write" not in auditor
    assert "attestations: write" not in auditor
    assert "attestations: read" in auditor
    assert "gh attestation verify" in auditor
    assert "uses: actions/upload-artifact@ea165f8d65b6e75b540449e92b4886f43607fa02 # v4.6.2" in auditor
    assert "d6u-trusted-input" not in signer
    assert "d6u-trusted-input" not in auditor
    assert "d6u-trusted-auditor-handoff-run-${{ github.run_id }}-attempt-${{ github.run_attempt }}" in verifier
    assert 'python3 scripts/integral/fetch_d6u_trusted_artifact.py \\' in auditor
    assert "--current-run-handoff" in auditor
    assert workflow.count("id-token: write") == 1
    assert workflow.count("attestations: write") == 1
    assert workflow.count("uses: actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2") == 1
