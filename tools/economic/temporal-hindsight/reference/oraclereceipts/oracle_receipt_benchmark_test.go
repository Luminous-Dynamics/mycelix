package oraclereceipts

import (
	"fmt"
	"testing"
)

func benchmarkOracleReceiptChain(b *testing.B, count int) (OracleScopeV1, []OracleReceiptV1, string) {
	b.Helper()
	scope := OracleScopeV1{
		SourceRef: "source:benchmark",
		SnapshotRef: "snapshot:benchmark",
		RegistryRef: "registry:benchmark",
		RegistryEpoch: "epoch:1",
		QueryProfileRef: "query:benchmark",
		QueryDigest: testDigestA,
		InputCommitmentDigest: testDigestB,
		EvidenceFrontierDigest: testDigestC,
	}
	receipts := make([]OracleReceiptV1, 0, count)
	previous := testGenesisBenchmark(scope)
	for i := 0; i < count; i++ {
		receipt := OracleReceiptV1{
			Sequence: uint64(i + 1),
			ReceiptID: fmt.Sprintf("receipt:%d", i),
			PreviousReceiptDigest: previous,
			OracleRef: "oracle:benchmark",
			KeyEpoch: "key:1",
			AuthorityGrantRef: "grant:benchmark",
			AuthorityScopeDigest: testDigestA,
			QueryDigest: scope.QueryDigest,
			InputCommitmentDigest: scope.InputCommitmentDigest,
			EvidenceFrontierDigest: scope.EvidenceFrontierDigest,
			AnswerSchemaRef: "answer:v1",
			AnswerDisposition: DispositionAffirm,
			AnswerPayload: []byte("benchmark answer"),
			AnswerDigest: AnswerDigestV1([]byte("benchmark answer")),
			RecordedAtUnixNanos: int64(i+1) * 100_000_000,
			ValidTime: ValidTimeV1{FromUnixNanos: 1, OpenEnded: true},
			EvidenceReferences: []EvidenceReferenceV1{{Reference: "evidence:1", Kind: "source", Digest: testDigestB}},
			SignatureScheme: "ed25519",
			SignatureBytes: []byte("structural-benchmark-signature"),
		}
		receipt.SignedMessageDigest, _ = SignableDigestV1(scope, receipt)
		receipt.ReceiptDigest, _ = ReceiptDigestV1(scope, receipt)
		receipts = append(receipts, receipt)
		previous = receipt.ReceiptDigest
	}
	return scope, receipts, previous
}

func testGenesisBenchmark(scope OracleScopeV1) string {
	scopeDigest, _ := ScopeDigestV1(scope)
	return genesisDigest(scopeDigest)
}

func BenchmarkVerifyOracleReceiptChain(b *testing.B) {
	for _, count := range []int{1, 10, 100, 1000, 10000} {
		b.Run(fmt.Sprintf("receipts=%d", count), func(b *testing.B) {
			scope, receipts, tail := benchmarkOracleReceiptChain(b, count)
			b.ReportAllocs()
			b.ReportMetric(float64(count), "receipts/op")
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				result := VerifyOracleReceiptChainV1(scope, receipts, uint64(count), tail)
				if result.Disposition != ChainStructurallyValid {
					b.Fatalf("receipt chain invalid: %s", result.Reason)
				}
			}
		})
	}
}
