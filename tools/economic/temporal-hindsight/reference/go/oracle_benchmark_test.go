package hindsight

import (
	"fmt"
	"testing"
)

func benchmarkDataset(ownerCount, intervalsPerOwner int) ([]string, map[string][]Event, Cut, Scope) {
	scope := Scope{Verifier: "verifier:bench", Policy: "policy:bench", Namespace: "ns:bench", Subject: "subject:bench", Claim: "claim:bench"}
	owners := make([]string, ownerCount)
	histories := make(map[string][]Event, ownerCount)
	cut := make(Cut, ownerCount)
	for oi := 0; oi < ownerCount; oi++ {
		owner := fmt.Sprintf("owner:%06d", oi)
		historyRef := fmt.Sprintf("history:%06d:v1", oi)
		owners[oi] = owner
		events := make([]Event, 0, intervalsPerOwner*2)
		for ii := 0; ii < intervalsPerOwner; ii++ {
			ref := fmt.Sprintf("interval:%08d", ii)
			base := uint64(ii*2 + 1)
			events = append(events,
				Event{Owner: owner, OwnerHistoryRef: historyRef, Namespace: scope.Namespace, Sequence: base, ID: fmt.Sprintf("%s:start", ref), Subject: scope.Subject, Claim: scope.Claim, IntervalRef: ref, Dimension: ValidFrom, Value: int64(ii * 10)},
				Event{Owner: owner, OwnerHistoryRef: historyRef, Namespace: scope.Namespace, Sequence: base + 1, ID: fmt.Sprintf("%s:end", ref), Subject: scope.Subject, Claim: scope.Claim, IntervalRef: ref, Dimension: ValidTo, Value: int64(ii*10 + 5)},
			)
		}
		histories[owner] = events
		cut[owner] = OwnerCut{OwnerHistoryRef: historyRef, Sequence: uint64(len(events))}
	}
	return owners, histories, cut, scope
}

func BenchmarkReplayByOwnerCount(b *testing.B) {
	for _, count := range []int{1, 10, 100, 1000} {
		b.Run(fmt.Sprintf("owners=%d", count), func(b *testing.B) {
			owners, histories, cut, scope := benchmarkDataset(count, 10)
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				result := Evaluate(owners, histories, cut, scope, 25)
				if result.Disposition != "CompleteForSuppliedOwnerPrefixes" {
					b.Fatal(result.Reason)
				}
			}
		})
	}
}

func BenchmarkReplayByIntervalCount(b *testing.B) {
	for _, count := range []int{100, 1000, 10000} {
		b.Run(fmt.Sprintf("intervals=%d", count), func(b *testing.B) {
			owners, histories, cut, scope := benchmarkDataset(1, count)
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				result := Evaluate(owners, histories, cut, scope, 25)
				if result.Disposition != "CompleteForSuppliedOwnerPrefixes" {
					b.Fatal(result.Reason)
				}
			}
		})
	}
}
