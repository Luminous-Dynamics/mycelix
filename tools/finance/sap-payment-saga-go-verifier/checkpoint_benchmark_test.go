package sagaverifier

import (
	"fmt"
	"testing"
)

func checkpointBenchmarkChain(tb testing.TB, owner CheckpointOwnerDomain, count int) []JournalCheckpointSubject {
	tb.Helper()
	intent := replayTestIntent()
	subjects := make([]JournalCheckpointSubject, 0, count)
	previousCheckpointDigest := ""
	previousActionHash := checkpointTestActionHash(41, 0)

	for index := 0; index < count; index++ {
		actionSequence := uint32(index + 42)
		currentActionHash := checkpointTestActionHash(
			byte(actionSequence),
			byte(actionSequence>>8),
		)
		subject := checkpointTestSubject(intent, owner)
		subject.EventCount = uint64(index + 1)
		subject.TailDigest = fmt.Sprintf("sha256:%064x", index+1)
		subject.CheckpointSequence = uint64(index + 1)
		if index > 0 {
			subject.PreviousCheckpointDigest = previousCheckpointDigest
			subject.HasPreviousCheckpoint = true
		}
		subject.SourceActionHash = currentActionHash
		subject.SourceActionSequence = actionSequence
		subject.SourcePreviousActionHash = append([]byte(nil), previousActionHash...)
		subject.HasSourcePreviousAction = true

		digest, err := subject.CanonicalDigest()
		if err != nil {
			tb.Fatalf("build checkpoint %d: %v", index, err)
		}
		previousCheckpointDigest = digest
		previousActionHash = currentActionHash
		subjects = append(subjects, subject)
	}
	return subjects
}

// BenchmarkCheckpointDigest measures only canonical subject validation and
// hashing. It excludes synthetic chain construction and all DHT/host I/O.
func BenchmarkCheckpointDigest(b *testing.B) {
	for _, owner := range []struct {
		name  string
		value CheckpointOwnerDomain
	}{
		{name: "payments", value: CheckpointPayments},
		{name: "treasury", value: CheckpointTreasury},
	} {
		for _, count := range []int{100, 1_000, 10_000} {
			subjects := checkpointBenchmarkChain(b, owner.value, count)
			b.Run(fmt.Sprintf("%s_%d", owner.name, count), func(b *testing.B) {
				b.ReportAllocs()
				b.ResetTimer()
				for iteration := 0; iteration < b.N; iteration++ {
					for _, subject := range subjects {
						if _, err := subject.CanonicalDigest(); err != nil {
							b.Fatalf("checkpoint digest: %v", err)
						}
					}
				}
			})
		}
	}
}
