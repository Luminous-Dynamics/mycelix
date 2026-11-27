package temporalrange

import "testing"

func TestFrontierProjectionReplayRequiresEveryOwnerAndCommittedAudit(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	binding.ProjectionAuditDigest = "sha256:" + fmtHexV1(audit.AuditDigest)

	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef,
		PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest,
		IntervalProfile: intervalProfileV1,
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{binding},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	result, err := VerifyTemporalFrontierProjectionReplayV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:grants"}, root, manifest, []AcceptedReceiptInventoryV1{inventory},
	)
	if err != nil {
		t.Fatal(err)
	}
	if result.Disposition != ProjectionMatchesAllSuppliedOwnerInventoriesV1 {
		t.Fatalf("wrong projection disposition: %s", result.Disposition)
	}
	if result.ReceiptInventoryAuthentication != ReceiptLogRootRecomputedButTrustExternalV1 {
		t.Fatalf("owner log root was not independently reconstructed: %s", result.ReceiptInventoryAuthentication)
	}
	if result.ProjectionMappingAuthentication != ProjectionMappingAuthenticationNotEvaluatedV1 {
		t.Fatalf("projection semantics were overclaimed: %s", result.ProjectionMappingAuthentication)
	}
	if result.OwnerCount != 1 || !result.HasOwnerGaps || len(result.OwnerAudits) != 1 {
		t.Fatalf("unexpected owner projection result: %+v", result)
	}

	if _, err := VerifyTemporalFrontierProjectionReplayV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:grants", "owner:missing"}, root, manifest, []AcceptedReceiptInventoryV1{inventory},
	); err != ErrForestProjectionOwnerSetMismatch {
		t.Fatalf("omitted frontier owner did not fail closed: %v", err)
	}

	changedInventory := inventory
	changedInventory.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	changedInventory.Receipts[0].ValidTo = intervalFiniteV1(31, 0)
	if _, err := VerifyTemporalFrontierProjectionReplayV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:grants"}, root, manifest, []AcceptedReceiptInventoryV1{changedInventory},
	); err != ErrProjectionIndexRootMismatch {
		t.Fatalf("omitted/altered projection did not fail closed: %v", err)
	}

	wrongRoot := [32]byte{0xAA}
	if _, err := VerifyTemporalFrontierProjectionReplayV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:grants"}, wrongRoot, manifest, []AcceptedReceiptInventoryV1{inventory},
	); err != ErrForestRootMismatch {
		t.Fatalf("untrusted forest root substitution: %v", err)
	}
}

func fmtHexV1(value [32]byte) string {
	const hex = "0123456789abcdef"
	out := make([]byte, 64)
	for i, b := range value {
		out[i*2] = hex[b>>4]
		out[i*2+1] = hex[b&0x0F]
	}
	return string(out)
}
