package oracle

import (
	"crypto/sha256"
	"encoding/hex"
	"testing"
)

func digest(label string) string {
	sum := sha256.Sum256([]byte(label))
	return hex.EncodeToString(sum[:])
}

func entry(id, stream string, epoch, offset uint64, label string, parents ...string) Entry {
	return Entry{ID: id, StreamID: stream, Epoch: epoch, Offset: offset, Digest: digest(label), Parents: parents}
}

func cut(stream string, epoch, offset uint64, head string) StreamCut {
	return StreamCut{StreamID: stream, Epoch: epoch, Offset: offset, HeadDigest: head}
}

func TestConcurrentEqualTimestampsRemainIncomparable(t *testing.T) {
	a := entry("a1", "alpha", 1, 1, "a1")
	b := entry("b1", "beta", 1, 1, "b1")
	a.ObservedAt = "2026-11-28T10:00:00Z"
	b.ObservedAt = "2026-11-28T10:00:00Z"
	p := Validate([]Entry{b, a}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, a.Digest), cut("beta", 1, 1, b.Digest)}})
	if p.Receipt.Status != StatusValid {
		t.Fatalf("status = %s, failures = %#v", p.Receipt.Status, p.Receipt.Failures)
	}
	if got := p.Compare("a1", "b1"); got != OrderIncomparable {
		t.Fatalf("same timestamp invented ordering: got %s", got)
	}
	if p.Receipt.SourceUniverseCompleteness != "NotEvaluated" {
		t.Fatalf("source universe must remain NotEvaluated, got %q", p.Receipt.SourceUniverseCompleteness)
	}
}

func TestVerifiedParentCreatesCausalOrder(t *testing.T) {
	parent := entry("parent", "alpha", 1, 1, "parent")
	child := entry("child", "beta", 1, 1, "child", "parent")
	p := Validate([]Entry{child, parent}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, parent.Digest), cut("beta", 1, 1, child.Digest)}})
	if p.Receipt.Status != StatusValid {
		t.Fatalf("status = %s, failures = %#v", p.Receipt.Status, p.Receipt.Failures)
	}
	if got := p.Compare("parent", "child"); got != OrderBefore {
		t.Fatalf("expected parent before child, got %s", got)
	}
}

func TestMissingParentMakesCutIncomplete(t *testing.T) {
	child := entry("child", "beta", 1, 1, "child", "not-in-cut")
	p := Validate([]Entry{child}, Frontier{Cuts: []StreamCut{cut("beta", 1, 1, child.Digest)}})
	if p.Receipt.Status != StatusIncomplete {
		t.Fatalf("expected incomplete, got %s", p.Receipt.Status)
	}
}

func TestForkAtSameStreamOffsetFailsClosed(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a")
	b := entry("b", "alpha", 1, 1, "b")
	p := Validate([]Entry{a, b}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, a.Digest)}})
	if p.Receipt.Status != StatusInvalid {
		t.Fatalf("expected invalid fork, got %s", p.Receipt.Status)
	}
}

func TestHeadDigestMismatchFailsClosed(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a")
	p := Validate([]Entry{a}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, digest("other"))}})
	if p.Receipt.Status != StatusInvalid {
		t.Fatalf("expected invalid head mismatch, got %s", p.Receipt.Status)
	}
}

func TestMissingPrefixOffsetIsIncomplete(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a")
	c := entry("c", "alpha", 1, 3, "c")
	p := Validate([]Entry{a, c}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 3, c.Digest)}})
	if p.Receipt.Status != StatusIncomplete {
		t.Fatalf("expected incomplete prefix, got %s", p.Receipt.Status)
	}
}

func TestFutureMalformedPayloadDoesNotPoisonHistoricalFrontier(t *testing.T) {
	known := entry("known", "alpha", 1, 1, "known")
	before := Validate([]Entry{known}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, known.Digest)}})
	future := entry("future", "alpha", 1, 2, "future", "missing-future-parent")
	future.Digest = "not-a-digest"
	after := Validate([]Entry{known, future}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, known.Digest)}})
	if before.Receipt.ReceiptSHA256 != after.Receipt.ReceiptSHA256 {
		t.Fatalf("later malformed payload changed historical receipt: %s != %s", before.Receipt.ReceiptSHA256, after.Receipt.ReceiptSHA256)
	}
}

func TestInputPermutationDoesNotChangeReceipt(t *testing.T) {
	a1 := entry("a1", "alpha", 1, 1, "a1")
	a2 := entry("a2", "alpha", 1, 2, "a2", "a1")
	b1 := entry("b1", "beta", 1, 1, "b1")
	f := Frontier{Cuts: []StreamCut{cut("alpha", 1, 2, a2.Digest), cut("beta", 1, 1, b1.Digest)}}
	first := Validate([]Entry{a1, a2, b1}, f)
	second := Validate([]Entry{b1, a2, a1}, f)
	if first.Receipt.ReceiptSHA256 != second.Receipt.ReceiptSHA256 {
		t.Fatalf("input order changed receipt: %s != %s", first.Receipt.ReceiptSHA256, second.Receipt.ReceiptSHA256)
	}
}

func TestSameStreamParentMustPrecedeChild(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a", "b")
	b := entry("b", "alpha", 1, 2, "b")
	p := Validate([]Entry{a, b}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 2, b.Digest)}})
	if p.Receipt.Status != StatusInvalid {
		t.Fatalf("expected non-causal parent failure, got %s", p.Receipt.Status)
	}
}

func TestCycleFailsClosed(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a", "b")
	b := entry("b", "beta", 1, 1, "b", "a")
	p := Validate([]Entry{a, b}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, a.Digest), cut("beta", 1, 1, b.Digest)}})
	if p.Receipt.Status != StatusInvalid {
		t.Fatalf("expected cycle failure, got %s", p.Receipt.Status)
	}
}

func TestReceiptBindsEveryIncludedEntryNotOnlyStreamHeads(t *testing.T) {
	a1 := entry("a1", "alpha", 1, 1, "a1")
	a2 := entry("a2", "alpha", 1, 2, "a2", "a1")
	f := Frontier{Cuts: []StreamCut{cut("alpha", 1, 2, a2.Digest)}}
	first := Validate([]Entry{a1, a2}, f)
	mutated := a1
	mutated.Digest = digest("changed-a1")
	second := Validate([]Entry{mutated, a2}, f)
	if first.Receipt.ReceiptSHA256 == second.Receipt.ReceiptSHA256 {
		t.Fatal("changing an included non-head entry did not change the receipt")
	}
	if first.Receipt.IncludedEntriesSHA256 == second.Receipt.IncludedEntriesSHA256 {
		t.Fatal("entry-set commitment did not change after mutation")
	}
}

func TestEventComparesEqualToItself(t *testing.T) {
	a := entry("a", "alpha", 1, 1, "a")
	p := Validate([]Entry{a}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, a.Digest)}})
	if got := p.Compare("a", "a"); got != OrderEqual {
		t.Fatalf("self comparison should be Equal, got %s", got)
	}
}

func TestWrongEpochCannotSatisfySelectedStreamPrefix(t *testing.T) {
	newEpoch := entry("new-epoch", "alpha", 2, 1, "new-epoch")
	p := Validate([]Entry{newEpoch}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, digest("old-epoch-head"))}})
	if p.Receipt.Status != StatusIncomplete {
		t.Fatalf("wrong-epoch entry must not satisfy selected prefix, got %s", p.Receipt.Status)
	}
}

func TestDuplicateEventIdentityFailsClosed(t *testing.T) {
	a := entry("same-id", "alpha", 1, 1, "a")
	b := entry("same-id", "beta", 1, 1, "b")
	p := Validate([]Entry{a, b}, Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, a.Digest), cut("beta", 1, 1, b.Digest)}})
	if p.Receipt.Status != StatusInvalid {
		t.Fatalf("duplicate global event identity must fail closed, got %s", p.Receipt.Status)
	}
}

func TestCanonicalEmptyParentSetProducesStableReceipt(t *testing.T) {
	nilParents := entry("a", "alpha", 1, 1, "a")
	emptyParents := nilParents
	emptyParents.Parents = []string{}
	f := Frontier{Cuts: []StreamCut{cut("alpha", 1, 1, nilParents.Digest)}}
	first := Validate([]Entry{nilParents}, f)
	second := Validate([]Entry{emptyParents}, f)
	if first.Receipt.ReceiptSHA256 != second.Receipt.ReceiptSHA256 {
		t.Fatalf("nil and empty parent set changed canonical receipt: %s != %s", first.Receipt.ReceiptSHA256, second.Receipt.ReceiptSHA256)
	}
}
