package sagaverifier

import (
	"fmt"
	"testing"
)

// replayBenchmarkHistory builds a deterministic, valid journal history with
// exactly eventCount frames. Repeated Treasury Indeterminate -> same-effect
// Prepared transitions model an acknowledgement-loss/reconciliation loop;
// no external effects are executed by this benchmark.
func replayBenchmarkHistory(tb testing.TB, eventCount int) ([]JournalEventFrame, ReplayAnchor) {
	tb.Helper()
	if eventCount < 6 || (eventCount-6)%2 != 0 {
		tb.Fatalf("event count must be 6 + 2*n, got %d", eventCount)
	}

	intent := replayTestIntent()
	journal := newJournal(intent)
	log, err := NewJournalEventLog(intent)
	if err != nil {
		t.Fatalf("create event log: %v", err)
	}
	if err := journal.Authorize(); err != nil {
		t.Fatalf("authorize journal: %v", err)
	}
	if err := log.RecordAuthorized(); err != nil {
		t.Fatalf("record authorization: %v", err)
	}

	localID := journal.EffectID(LocalPaymentTransaction)
	if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
		t.Fatalf("prepare local leg: %v", err)
	}
	if err := log.RecordEffectPrepared(LocalPaymentTransaction, localID); err != nil {
		t.Fatalf("record local prepare: %v", err)
	}
	localEvidence := replayTestLocalEvidence(journal)
	if _, err := journal.CommitLeg(LocalPaymentTransaction, localEvidence); err != nil {
		t.Fatalf("commit local leg: %v", err)
	}
	if err := log.RecordEffectCommitted(localEvidence); err != nil {
		t.Fatalf("record local commit: %v", err)
	}

	treasuryID := journal.EffectID(TreasuryFee)
	if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
		t.Fatalf("prepare Treasury leg: %v", err)
	}
	if err := log.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
		t.Fatalf("record Treasury prepare: %v", err)
	}

	retryCycles := (eventCount - 6) / 2
	for cycle := 0; cycle < retryCycles; cycle++ {
		if err := journal.MarkIndeterminate(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("mark Treasury effect indeterminate at cycle %d: %v", cycle, err)
		}
		if err := log.RecordEffectIndeterminate(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("record indeterminate Treasury effect at cycle %d: %v", cycle, err)
		}
		if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("resume same Treasury effect at cycle %d: %v", cycle, err)
		}
		if err := log.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("record same-effect retry at cycle %d: %v", cycle, err)
		}
	}

	treasuryEvidence := replayTestTreasuryEvidence(journal)
	if _, err := journal.CommitLeg(TreasuryFee, treasuryEvidence); err != nil {
		t.Fatalf("commit Treasury leg: %v", err)
	}
	if err := log.RecordEffectCommitted(treasuryEvidence); err != nil {
		t.Fatalf("record Treasury commit: %v", err)
	}

	frames := log.Frames()
	if len(frames) != eventCount {
		t.Fatalf("benchmark history has %d frames, want %d", len(frames), eventCount)
	}
	if status := journal.Status(); status != Completed {
		tb.Fatalf("benchmark history status = %q, want %q", status, Completed)
	}
	return frames, log.Anchor()
}

// BenchmarkReplayJournal measures only in-memory validation and reconstruction.
// It excludes event-log construction, host I/O, Holochain DHT retrieval and
// external payment/Treasury effects.
func BenchmarkReplayJournal(b *testing.B) {
	for _, eventCount := range []int{100, 1_000, 10_000} {
		frames, anchor := replayBenchmarkHistory(b, eventCount)
		b.Run(fmt.Sprintf("events_%d", eventCount), func(b *testing.B) {
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if _, err := ReplayJournal(frames, anchor); err != nil {
					b.Fatalf("replay failed: %v", err)
				}
			}
		})
	}
}
