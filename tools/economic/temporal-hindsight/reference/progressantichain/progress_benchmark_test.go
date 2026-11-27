package progressantichain

import (
	"fmt"
	"testing"
)

func benchmarkProgressFixture(b *testing.B, ownerCount, width int) (ProgressAntichainV1, ProgressCutV1) {
	b.Helper()
	owners := make([]string, ownerCount)
	query := ProgressCutV1{Owners: make([]OwnerSequenceV1, ownerCount)}
	for i := range owners {
		owners[i] = fmt.Sprintf("owner:%05d", i)
		query.Owners[i] = OwnerSequenceV1{OwnerRef: owners[i], Sequence: 0}
	}
	scope := ProgressScopeV1{
		SourceRef: "directory:benchmark",
		RegistryRef: "registry:benchmark",
		RegistryEpoch: "epoch:1",
		ProfileRef: "universe:benchmark",
		QueryScopeDigest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		OwnerRefs: owners,
	}
	antichain := ProgressAntichainV1{Scope: scope, Points: make([]ProgressCutV1, width)}
	for point := 0; point < width; point++ {
		cut := ProgressCutV1{Owners: make([]OwnerSequenceV1, ownerCount)}
		for owner, ownerRef := range owners {
			sequence := uint64(0)
			if owner == point {
				sequence = 1
			}
			cut.Owners[owner] = OwnerSequenceV1{OwnerRef: ownerRef, Sequence: sequence}
		}
		antichain.Points[point] = cut
	}
	return antichain, query
}

func benchmarkProgressCases(b *testing.B, operation string, run func(*testing.B, ProgressAntichainV1, ProgressCutV1)) {
	for _, ownerCount := range []int{1, 10, 100, 1000, 10000} {
		for _, width := range []int{0, 1, 2, 4, 8, 16, 64, 256} {
			if width > ownerCount {
				continue
			}
			name := fmt.Sprintf("%s/owners=%d/width=%d", operation, ownerCount, width)
			b.Run(name, func(b *testing.B) {
				antichain, query := benchmarkProgressFixture(b, ownerCount, width)
				run(b, antichain, query)
				b.ReportMetric(float64(ownerCount), "owners/op")
				b.ReportMetric(float64(width), "points/op")
			})
		}
	}
}

func BenchmarkValidateProgressAntichain(b *testing.B) {
	benchmarkProgressCases(b, "validate", func(b *testing.B, antichain ProgressAntichainV1, _ ProgressCutV1) {
		b.ReportAllocs()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			if _, err := ValidateProgressAntichainV1(antichain); err != nil {
				b.Fatalf("antichain validation failed: %v", err)
			}
		}
	})
}

func BenchmarkEvaluateProgressAt(b *testing.B) {
	benchmarkProgressCases(b, "evaluate", func(b *testing.B, antichain ProgressAntichainV1, query ProgressCutV1) {
		b.ReportAllocs()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			result := EvaluateProgressAtV1(antichain, query)
			if result.Disposition == ProgressIndeterminate {
				b.Fatalf("progress evaluation failed: %s", result.Reason)
			}
		}
	})
}

func BenchmarkAdvanceProgressAntichain(b *testing.B) {
	benchmarkProgressCases(b, "advance", func(b *testing.B, next ProgressAntichainV1, _ ProgressCutV1) {
		ownerCount := len(next.Scope.OwnerRefs)
		previousPoint := ProgressCutV1{Owners: make([]OwnerSequenceV1, ownerCount)}
		for i, owner := range next.Scope.OwnerRefs {
			previousPoint.Owners[i] = OwnerSequenceV1{OwnerRef: owner, Sequence: 0}
		}
		previous := ProgressAntichainV1{Scope: next.Scope, Points: []ProgressCutV1{previousPoint}}
		b.ReportAllocs()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			result := AdvanceProgressAntichainV1(previous, next)
			if result.Disposition != AdvanceMonotone {
				b.Fatalf("progress advance failed: %s", result.Reason)
			}
		}
	})
}
