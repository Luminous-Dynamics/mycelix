package temporalrange

import (
	"fmt"
	"testing"
)

type frontierForestBenchmarkFixtureV1 struct {
	scope       IntervalScopeV1
	owners      []string
	manifest    TemporalFrontierIndexManifestV1
	proof       TemporalFrontierQueryProofV1
	root        [32]byte
	proofBytes  int
	recordCount int
}

func makeFrontierForestBenchmarkFixtureV1(ownerCount int) frontierForestBenchmarkFixtureV1 {
	scope := intervalScopeFixtureV1()
	query := intervalPointV1(50, 0)
	fixture := frontierForestBenchmarkFixtureV1{
		scope:  scope,
		owners: make([]string, 0, ownerCount),
		manifest: TemporalFrontierIndexManifestV1{
			VerifierRef:     scope.VerifierRef,
			PolicyEpochRef:  scope.PolicyEpochRef,
			FrontierDigest:  scope.SourceFrontierDigest,
			IntervalProfile: intervalProfileV1,
			QueryScope:      scope,
			Owners:          make([]OwnerIntervalIndexBindingV1, 0, ownerCount),
		},
		proof: TemporalFrontierQueryProofV1{
			Query:       query,
			OwnerProofs: make([]OwnerIntervalQueryProofV1, 0, ownerCount),
		},
		recordCount: ownerCount,
	}
	for i := 0; i < ownerCount; i++ {
		ownerRef := fmt.Sprintf("owner:%08d", i)
		historyRef := fmt.Sprintf("history:%08d", i)
		record := IntervalRecordV1{
			AssertionRef:  fmt.Sprintf("assertion:%08d", i),
			ValidFrom:     intervalPointV1(0, 0),
			ValidTo:       intervalFiniteV1(100, 0),
			ReceiptDigest: intervalDigestV1(byte((i % 250) + 1)),
		}
		tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{record})
		if err != nil {
			panic(err)
		}
		binding := OwnerIntervalIndexBindingV1{
			OwnerRef:              ownerRef,
			OwnerHistoryRef:       historyRef,
			OwnerSequence:         1,
			TreeSize:              1,
			OwnerLogRootDigest:    fmt.Sprintf("sha256:%064x", uint64(i+1)),
			IntervalIndexRoot:     tree.RootHash(),
			ProjectionAuditDigest: fmt.Sprintf("sha256:%064x", uint64(i+1001)),
		}
		proof, err := tree.ProvePointQueryV1(query)
		if err != nil {
			panic(err)
		}
		encoded, err := MarshalIntervalPointProofV1(proof)
		if err != nil {
			panic(err)
		}
		fixture.proofBytes += len(encoded)
		fixture.owners = append(fixture.owners, ownerRef)
		fixture.manifest.Owners = append(fixture.manifest.Owners, binding)
		fixture.proof.OwnerProofs = append(fixture.proof.OwnerProofs, OwnerIntervalQueryProofV1{
			OwnerRef: ownerRef,
			Proof:    proof,
		})
	}
	fixture.proof.Manifest = fixture.manifest
	root, err := TemporalFrontierForestRootV1(fixture.manifest)
	if err != nil {
		panic(err)
	}
	fixture.root = root
	return fixture
}

func BenchmarkFrontierForestV1(b *testing.B) {
	for _, owners := range []int{1, 10, 100, 1_000} {
		b.Run(fmt.Sprintf("verify/%d_owners", owners), func(b *testing.B) {
			fixture := makeFrontierForestBenchmarkFixtureV1(owners)
			b.ReportMetric(float64(owners), "owners")
			b.ReportMetric(float64(fixture.proofBytes), "owner_proof_B")
			b.ReportMetric(float64(fixture.recordCount), "records")
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				result, err := VerifyTemporalFrontierQueryProofV1(
					fixture.scope.VerifierRef,
					fixture.scope.PolicyEpochRef,
					fixture.scope.SourceFrontierDigest,
					fixture.scope,
					fixture.owners,
					fixture.proof.Query,
					fixture.proof,
					fixture.root,
				)
				if err != nil || result.OwnerCount != owners {
					b.Fatalf("forest verification failed: owners=%d result=%+v err=%v", owners, result, err)
				}
			}
		})
		b.Run(fmt.Sprintf("build/%d_owners", owners), func(b *testing.B) {
			b.ReportMetric(float64(owners), "owners")
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				fixture := makeFrontierForestBenchmarkFixtureV1(owners)
				if fixture.root == ([32]byte{}) || len(fixture.proof.OwnerProofs) != owners {
					b.Fatalf("forest build failed for %d owners", owners)
				}
			}
		})
	}
}
