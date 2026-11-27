package temporalrange

import (
	"crypto/sha256"
	"fmt"
	"testing"
)

func BenchmarkOwnerProjectionReplayV1(b *testing.B) {
	for _, size := range []int{100, 1_000, 10_000, 100_000} {
		scope := intervalScopeFixtureV1()
		owner := "owner:projection-benchmark"
		history := "history:projection-benchmark"
		records := make([]IntervalRecordV1, size)
		receipts := make([]AcceptedReceiptProjectionV1, size)
		leaves := make([]OwnerLogLeafV1, size)
		logEntries := make([][]byte, size)
		for i := 0; i < size; i++ {
			assertionID := fmt.Sprintf("assertion:%08d", i)
			assertionPayload := []byte(fmt.Sprintf(`{"profile":"MYCELIX-INTERVAL-ASSERTION-BENCH-V1","assertion_id":"%s","valid_from":%d,"valid_to":%d}`, assertionID, i, i+100))
			assertionDigestBytes := sha256.Sum256(assertionPayload)
			assertionDigest := fmt.Sprintf("sha256:%x", assertionDigestBytes)
			sequence := uint64(i + 1)
			entry := canonicalReceiptFixtureV1(scope, fmt.Sprintf("receipt:%08d", i), assertionID, assertionDigest, owner, sequence)
			receiptDigest := receiptDigestFixtureV1(entry)
			record := IntervalRecordV1{
				AssertionRef: assertionID,
				ValidFrom: intervalPointV1(int64(i), 0),
				ValidTo: intervalFiniteV1(int64(i+100), 0),
				ReceiptDigest: receiptDigest,
			}
			records[i] = record
			receipts[i] = AcceptedReceiptProjectionV1{
				ReceiptLogLeafIndex: sequence - 1,
				OwnerSequence: sequence,
				ReceiptDigest: receiptDigest,
				AssertionDigest: assertionDigest,
				AssertionPayloadBytes: assertionPayload,
				CanonicalReceiptEntryBytes: entry,
				AssertionID: assertionID,
				Class: ReceiptProjectsIntervalAssertionV1,
				AssertionRef: assertionID,
				ValidFrom: record.ValidFrom,
				ValidTo: record.ValidTo,
			}
			leaves[i] = OwnerLogLeafV1{LeafIndex: sequence - 1, OwnerSequence: sequence, EntryBytes: entry}
			logEntries[i] = entry
		}
		tree, err := NewIntervalTreeV1(scope, records)
		if err != nil {
			b.Fatal(err)
		}
		binding := OwnerIntervalIndexBindingV1{
			OwnerRef: owner,
			OwnerHistoryRef: history,
			OwnerSequence: uint64(size),
			TreeSize: uint64(size),
			OwnerLogRootDigest: fmt.Sprintf("sha256:%x", rfc9162TreeHashV1(logEntries)),
			IntervalIndexRoot: tree.RootHash(),
			ProjectionAuditDigest: "",
		}
		inventory := AcceptedReceiptInventoryV1{
			OwnerRef: owner,
			OwnerHistoryRef: history,
			OwnerSequenceCut: uint64(size),
			TreeSize: uint64(size),
			OwnerLogRootDigest: binding.OwnerLogRootDigest,
			LogLeaves: leaves,
			Receipts: receipts,
		}

		b.Run(fmt.Sprintf("replay/%d_receipts", size), func(b *testing.B) {
			b.ReportMetric(float64(size), "receipts")
			b.ReportMetric(float64(size), "log_leaves")
			b.ReportMetric(float64(len(logEntries[size-1]))*float64(size), "approx_log_bytes")
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
				if err != nil || !audit.OwnerLogRootMatches || audit.IntervalAssertionCount != uint64(size) {
					b.Fatalf("projection replay failed: %+v %v", audit, err)
				}
			}
		})
	}
}
