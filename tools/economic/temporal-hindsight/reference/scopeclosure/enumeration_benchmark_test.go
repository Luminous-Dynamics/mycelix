package scopeclosure

import (
	"fmt"
	"testing"
)

func benchmarkEnumerationFixture(b *testing.B, pageCount int) (EnumerationManifestV1, []EnumerationPageV1) {
	b.Helper()
	pages := make([]EnumerationPageV1, pageCount)
	all := make([]OwnerCandidate, 0, pageCount)
	for i := 0; i < pageCount; i++ {
		candidate := OwnerCandidate{
			OwnerRef: fmt.Sprintf("owner:%08d", i),
			HistoryRef: fmt.Sprintf("history:%08d:v1", i),
			Disposition: CandidateRequired,
		}
		all = append(all, candidate)
		page := EnumerationPageV1{
			SourceRef: "directory:benchmark",
			SnapshotRef: "snapshot:benchmark",
			RegistryRef: "registry:benchmark",
			RegistryEpoch: "epoch:1",
			ProfileRef: "universe:benchmark",
			ScopeDigest: fmt.Sprintf("%064x", 1),
			PredicateDigest: fmt.Sprintf("%064x", 2),
			PageIndex: uint64(i),
			Candidates: []OwnerCandidate{candidate},
			Terminal: i == pageCount-1,
		}
		if i > 0 {
			page.RequestCursorDigest = fmt.Sprintf("%064x", i+100)
		}
		if i < pageCount-1 {
			page.NextCursorDigest = fmt.Sprintf("%064x", i+101)
		}
		digest, err := EnumerationPageDigestV1(page)
		if err != nil {
			b.Fatal(err)
		}
		page.PageDigest = digest
		pages[i] = page
	}
	root, err := CandidateRoot(all)
	if err != nil {
		b.Fatal(err)
	}
	manifest := EnumerationManifestV1{
		SourceRef: "directory:benchmark",
		SnapshotRef: "snapshot:benchmark",
		RegistryRef: "registry:benchmark",
		RegistryEpoch: "epoch:1",
		ProfileRef: "universe:benchmark",
		ScopeDigest: fmt.Sprintf("%064x", 1),
		PredicateDigest: fmt.Sprintf("%064x", 2),
		PageCount: uint64(pageCount),
		FirstPageDigest: pages[0].PageDigest,
		TerminalPageDigest: pages[len(pages)-1].PageDigest,
		CandidateRoot: root,
		SourceCompletionReceiptDigest: fmt.Sprintf("%064x", 3),
		CompletionDisposition: SourceDeclaredComplete,
	}
	return manifest, pages
}

// BenchmarkEnumerationPageChain measures in-memory structural validation only.
// It does not include network enumeration, signature/checkpoint verification,
// registry completeness verification, receipt verification or frontier replay.
func BenchmarkEnumerationPageChain(b *testing.B) {
	for _, pageCount := range []int{1, 10, 100, 1000, 10000} {
		b.Run(fmt.Sprintf("pages=%d", pageCount), func(b *testing.B) {
			manifest, pages := benchmarkEnumerationFixture(b, pageCount)
			b.ReportAllocs()
			b.ReportMetric(float64(pageCount), "pages/op")
			b.ReportMetric(float64(pageCount), "owners/op")
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				result := VerifyEnumerationPageChainV1(manifest, pages)
				if result.Disposition != EnumerationChainWellFormed {
					b.Fatalf("page chain failed: %s", result.Reason)
				}
			}
		})
	}
}
