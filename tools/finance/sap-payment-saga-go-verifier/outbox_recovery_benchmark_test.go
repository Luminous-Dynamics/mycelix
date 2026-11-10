package sagaverifier

import (
	"strconv"
	"testing"
)

func buildOutboxBenchmarkBook(b *testing.B, size int) *OutboxBook {
	b.Helper()
	book := NewOutboxBook()
	payload := "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	for index := 0; index < size; index++ {
		effect := NewOutboxEffect{
			OperationID:   "outbox-bench-operation-" + strconv.Itoa(index),
			EffectID:      "outbox-bench-treasury-fee-" + strconv.Itoa(index),
			DestinationID: "treasury:benchmark-destination",
			PayloadDigest: payload,
			CreatedAtMS:   1000,
		}
		if _, err := book.Enqueue(effect); err != nil {
			b.Fatal(err)
		}
	}
	return book
}

func outboxBenchmarkPolicy() BackoffPolicy {
	return BackoffPolicy{
		BaseDelayMS: 1, MaxDelayMS: 1000, JitterWindowMS: 10,
		MaxDispatchAttempts: 10, MaxReconciliationAttempts: 10,
		MaxReconciliationAgeMS: 60000,
	}
}

func BenchmarkOutboxClaimDue(b *testing.B) {
	for _, size := range []int{1000, 10000, 100000} {
		b.Run("items_"+strconv.Itoa(size), func(b *testing.B) {
			book := buildOutboxBenchmarkBook(b, size)
			policy := outboxBenchmarkPolicy()
			b.ReportAllocs()
			b.ResetTimer()
			now := uint64(1000)
			for i := 0; i < b.N; i++ {
				leases, err := book.ClaimDue("outbox-benchmark-worker", now, 1000, 64, policy)
				if err != nil {
					b.Fatal(err)
				}
				if len(leases) != 64 {
					b.Fatalf("claimed %d items, want 64", len(leases))
				}
				now += 1001
			}
		})
	}
}

func BenchmarkOutboxMetrics(b *testing.B) {
	for _, size := range []int{1000, 10000, 100000} {
		b.Run("items_"+strconv.Itoa(size), func(b *testing.B) {
			book := buildOutboxBenchmarkBook(b, size)
			b.ReportAllocs()
			b.ResetTimer()
			var now uint64 = 1500
			for i := 0; i < b.N; i++ {
				metrics := book.Metrics(now)
				if metrics.Ready != size {
					b.Fatalf("ready items = %d, want %d", metrics.Ready, size)
				}
				now++
			}
		})
	}
}
