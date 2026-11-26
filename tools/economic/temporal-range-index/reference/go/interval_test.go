package temporalrange

import (
	"crypto/sha256"
	"fmt"
	"testing"
)

func intervalDigestV1(b byte) string {
	return fmt.Sprintf("sha256:%064x", uint64(b))
}

func intervalPointV1(sec int64, ns uint32) IntervalInstantV1 {
	return IntervalInstantV1{Seconds: sec, Nanos: ns}
}

func intervalOpenV1() IntervalEndV1 {
	return IntervalEndV1{Open: true}
}

func intervalFiniteV1(sec int64, ns uint32) IntervalEndV1 {
	return IntervalEndV1{At: intervalPointV1(sec, ns)}
}

func intervalScopeFixtureV1() IntervalScopeV1 {
	return IntervalScopeV1{
		"did:mycelix:verifier:finance-v1",
		"policy-epoch:7",
		"urn:mycelix:grants",
		"subject:grant:42",
		"claim:active",
		"sha256:" + fmt.Sprintf("%064x", 99),
	}
}

func intervalRecordFixtureV1(id string, start, end int64) IntervalRecordV1 {
	return IntervalRecordV1{
		AssertionRef: id, ValidFrom: intervalPointV1(start, 0),
		ValidTo: intervalFiniteV1(end, 0), ReceiptDigest: intervalDigestV1(byte(start + 1)),
	}
}

func TestIntervalQueryHalfOpenEndpointsAndOpenEnd(t *testing.T) {
	scope := intervalScopeFixtureV1()
	records := []IntervalRecordV1{
		intervalRecordFixtureV1("a", 10, 20),
		{AssertionRef: "b", ValidFrom: intervalPointV1(15, 0), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(9)},
	}
	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	cases := []struct {
		q    IntervalInstantV1
		want []string
	}{
		{intervalPointV1(9, 999_999_999), nil},
		{intervalPointV1(10, 0), []string{"a"}},
		{intervalPointV1(19, 999_999_999), []string{"a", "b"}},
		{intervalPointV1(20, 0), []string{"b"}},
		{intervalPointV1(100, 0), []string{"b"}},
	}
	for _, tc := range cases {
		proof, err := tree.ProvePointQueryV1(tc.q)
		if err != nil {
			t.Fatal(err)
		}
		got, err := VerifyIntervalPointProofV1(scope, tc.q, proof, tree.RootHash())
		if err != nil {
			t.Fatalf("q=%+v: %v", tc.q, err)
		}
		if len(got.Records) != len(tc.want) {
			t.Fatalf("q=%+v got=%v want=%v", tc.q, got.Records, tc.want)
		}
		for i, want := range tc.want {
			if got.Records[i].AssertionRef != want {
				t.Fatalf("q=%+v index=%d got=%s want=%s", tc.q, i, got.Records[i].AssertionRef, want)
			}
		}
		if len(tc.want) == 0 && got.Disposition != CompleteEmptyIntervalResultForRoot {
			t.Fatalf("empty disposition: %s", got.Disposition)
		}
	}
}

func TestIntervalQueryNanosecondBoundary(t *testing.T) {
	scope := intervalScopeFixtureV1()
	record := IntervalRecordV1{
		AssertionRef: "nano", ValidFrom: intervalPointV1(1, 999_999_999),
		ValidTo: intervalFiniteV1(2, 1), ReceiptDigest: intervalDigestV1(7),
	}
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{record})
	if err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		q    IntervalInstantV1
		want bool
	}{
		{intervalPointV1(1, 999_999_998), false},
		{intervalPointV1(1, 999_999_999), true},
		{intervalPointV1(2, 0), true},
		{intervalPointV1(2, 1), false},
	} {
		proof, err := tree.ProvePointQueryV1(tc.q)
		if err != nil {
			t.Fatal(err)
		}
		got, err := VerifyIntervalPointProofV1(scope, tc.q, proof, tree.RootHash())
		if err != nil {
			t.Fatal(err)
		}
		if (len(got.Records) == 1) != tc.want {
			t.Fatalf("q=%+v got=%v want=%v", tc.q, got.Records, tc.want)
		}
	}
}

func TestIntervalProofRejectsScopeQueryAndRootSubstitution(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	q := intervalPointV1(5, 0)
	proof, _ := tree.ProvePointQueryV1(q)
	other := scope
	other.SubjectRef = "subject:other"
	if _, err := VerifyIntervalPointProofV1(other, q, proof, tree.RootHash()); err != ErrIntervalScopeMismatch {
		t.Fatalf("scope substitution: %v", err)
	}
	if _, err := VerifyIntervalPointProofV1(scope, intervalPointV1(6, 0), proof, tree.RootHash()); err != ErrIntervalQueryMismatch {
		t.Fatalf("query substitution: %v", err)
	}
	wrong := sha256.Sum256([]byte("wrong root"))
	if _, err := VerifyIntervalPointProofV1(scope, q, proof, wrong); err != ErrIntervalRootMismatch {
		t.Fatalf("root substitution: %v", err)
	}
}

func TestIntervalProofRejectsUnsafePruneAndMalformedNodes(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	q := intervalPointV1(5, 0)
	proof, _ := tree.ProvePointQueryV1(q)
	mutated := proof
	mutated.Tree = cloneIntervalProofNodeV1(proof.Tree)
	mutated.Tree.Kind = IntervalProofPrunedV1
	mutated.Tree.Record = nil
	mutated.Tree.Left = nil
	mutated.Tree.Right = nil
	mutated.Tree.Count = 1
	mutated.Tree.Summary = IntervalSummaryV1{
		MinValidFrom: intervalPointV1(1, 0), MaxValidTo: intervalFiniteV1(9, 0),
	}
	mutated.Tree.Hash = tree.rootHash
	if _, err := VerifyIntervalPointProofV1(scope, q, mutated, tree.RootHash()); err != ErrIntervalUnsafePrune {
		t.Fatalf("unsafe prune: %v", err)
	}
	mutated = proof
	mutated.Tree = cloneIntervalProofNodeV1(proof.Tree)
	mutated.Tree.Right = &IntervalProofNodeV1{Kind: IntervalProofLeafV1}
	if _, err := VerifyIntervalPointProofV1(scope, q, mutated, tree.RootHash()); err == nil {
		t.Fatal("malformed leaf passed")
	}
}

func TestIntervalRejectsInvalidIntervalsAndNanos(t *testing.T) {
	scope := intervalScopeFixtureV1()
	bad := intervalRecordFixtureV1("bad", 5, 5)
	if _, err := NewIntervalTreeV1(scope, []IntervalRecordV1{bad}); err == nil {
		t.Fatal("zero-length interval passed")
	}
	bad = intervalRecordFixtureV1("bad", 5, 4)
	if _, err := NewIntervalTreeV1(scope, []IntervalRecordV1{bad}); err == nil {
		t.Fatal("reversed interval passed")
	}
	bad = intervalRecordFixtureV1("bad", 5, 7)
	bad.ValidFrom.Nanos = 1_000_000_000
	if _, err := NewIntervalTreeV1(scope, []IntervalRecordV1{bad}); err == nil {
		t.Fatal("nanos overflow passed")
	}
	bad = intervalRecordFixtureV1("bad", 5, 7)
	bad.ValidTo = intervalOpenV1()
	bad.ValidTo.At = intervalPointV1(5, 0)
	if _, err := NewIntervalTreeV1(scope, []IntervalRecordV1{bad}); err == nil {
		t.Fatal("non-canonical open end passed")
	}
}

func TestIntervalEmptyRootProducesCompleteEmptyResult(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, nil)
	if err != nil {
		t.Fatal(err)
	}
	proof, err := tree.ProvePointQueryV1(intervalPointV1(1, 0))
	if err != nil {
		t.Fatal(err)
	}
	got, err := VerifyIntervalPointProofV1(scope, proof.Query, proof, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	if got.Disposition != CompleteEmptyIntervalResultForRoot || len(got.Records) != 0 {
		t.Fatalf("unexpected empty result: %+v", got)
	}
}

func TestIntervalTreeExactLinearScanParity(t *testing.T) {
	scope := intervalScopeFixtureV1()
	records := make([]IntervalRecordV1, 64)
	for i := range records {
		records[i] = intervalRecordFixtureV1(fmt.Sprintf("assertion:%03d", i), int64(i*3), int64(i*3+5))
	}
	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	for q := int64(0); q < 200; q++ {
		query := intervalPointV1(q, 0)
		proof, err := tree.ProvePointQueryV1(query)
		if err != nil {
			t.Fatal(err)
		}
		got, err := VerifyIntervalPointProofV1(scope, query, proof, tree.RootHash())
		if err != nil {
			t.Fatalf("q=%d: %v", q, err)
		}
		var want []string
		for _, record := range records {
			if intervalMatchesV1(record, query) {
				want = append(want, record.AssertionRef)
			}
		}
		if len(got.Records) != len(want) {
			t.Fatalf("q=%d got=%d want=%d", q, len(got.Records), len(want))
		}
		for i := range want {
			if got.Records[i].AssertionRef != want[i] {
				t.Fatalf("q=%d result[%d]=%s want=%s", q, i, got.Records[i].AssertionRef, want[i])
			}
		}
	}
}

func TestIntervalRootMatchesFrozenKnownAnswer(t *testing.T) {
	scope := intervalScopeFixtureV1()
	records := []IntervalRecordV1{
		{AssertionRef: "assertion:a", ValidFrom: intervalPointV1(10, 0), ValidTo: intervalFiniteV1(20, 0), ReceiptDigest: intervalDigestV1(1)},
		{AssertionRef: "assertion:b", ValidFrom: intervalPointV1(15, 0), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(2)},
		{AssertionRef: "assertion:c", ValidFrom: intervalPointV1(30, 0), ValidTo: intervalFiniteV1(40, 0), ReceiptDigest: intervalDigestV1(3)},
		{AssertionRef: "assertion:d", ValidFrom: intervalPointV1(15, 0), ValidTo: intervalFiniteV1(22, 0), ReceiptDigest: intervalDigestV1(4)},
	}
	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	if got := fmt.Sprintf("%x", tree.RootHash()); got != "a770c656c9afcafc3204006c5e8d217fc5ab38e66f90194e40c4cbdb3aabaac6" {
		t.Fatalf("interval root known-answer mismatch: %s", got)
	}
	proof, err := tree.ProvePointQueryV1(intervalPointV1(15, 0))
	if err != nil {
		t.Fatal(err)
	}
	result, err := VerifyIntervalPointProofV1(scope, proof.Query, proof, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	want := []string{"assertion:a", "assertion:b", "assertion:d"}
	if len(result.Records) != len(want) {
		t.Fatalf("known-answer result count: got=%d want=%d", len(result.Records), len(want))
	}
	for i, expected := range want {
		if result.Records[i].AssertionRef != expected {
			t.Fatalf("known-answer result[%d]: got=%s want=%s", i, result.Records[i].AssertionRef, expected)
		}
	}
}

func TestIntervalTieBreakAndAllOverlapCompleteness(t *testing.T) {
	scope := intervalScopeFixtureV1()
	records := []IntervalRecordV1{
		{AssertionRef: "z-last", ValidFrom: intervalPointV1(10, 0), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(1)},
		{AssertionRef: "b-middle", ValidFrom: intervalPointV1(10, 0), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(2)},
		{AssertionRef: "a-first", ValidFrom: intervalPointV1(10, 0), ValidTo: intervalOpenV1(), ReceiptDigest: intervalDigestV1(3)},
	}
	tree, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(10, 0)
	proof, err := tree.ProvePointQueryV1(query)
	if err != nil {
		t.Fatal(err)
	}
	result, err := VerifyIntervalPointProofV1(scope, query, proof, tree.RootHash())
	if err != nil {
		t.Fatal(err)
	}
	want := []string{"a-first", "b-middle", "z-last"}
	if len(result.Records) != len(want) {
		t.Fatalf("all-overlap count: got=%d want=%d", len(result.Records), len(want))
	}
	for i := range want {
		if result.Records[i].AssertionRef != want[i] {
			t.Fatalf("tie order[%d]: got=%s want=%s", i, result.Records[i].AssertionRef, want[i])
		}
	}
}

func TestIntervalScopeChangesRootEvenWithSameRecords(t *testing.T) {
	scope := intervalScopeFixtureV1()
	records := []IntervalRecordV1{intervalRecordFixtureV1("same-record", 1, 9)}
	first, err := NewIntervalTreeV1(scope, records)
	if err != nil {
		t.Fatal(err)
	}
	other := scope
	other.VerifierRef = "did:mycelix:verifier:another-v1"
	second, err := NewIntervalTreeV1(other, records)
	if err != nil {
		t.Fatal(err)
	}
	if first.RootHash() == second.RootHash() {
		t.Fatal("scoped roots collided across verifier identities")
	}
}

func BenchmarkAuthenticatedIntervalV1(b *testing.B) {
	for _, size := range []int{1_000, 10_000, 100_000} {
		b.Run(fmt.Sprintf("build/%d", size), func(b *testing.B) {
			scope := intervalScopeFixtureV1()
			records := make([]IntervalRecordV1, size)
			for i := range records {
				start := int64(i * 4)
				records[i] = intervalRecordFixtureV1(fmt.Sprintf("assertion:%08d", i), start, start+3)
			}
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if _, err := NewIntervalTreeV1(scope, records); err != nil {
					b.Fatal(err)
				}
			}
		})
		b.Run(fmt.Sprintf("linear_scan/%d", size), func(b *testing.B) {
			records := make([]IntervalRecordV1, size)
			for i := range records {
				start := int64(i * 4)
				records[i] = intervalRecordFixtureV1(fmt.Sprintf("assertion:%08d", i), start, start+3)
			}
			query := intervalPointV1(int64(size*2), 0)
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				var matches int
				for _, record := range records {
					if intervalMatchesV1(record, query) {
						matches++
					}
				}
				if matches != 1 {
					b.Fatalf("scan match count=%d want=1", matches)
				}
			}
		})
		b.Run(fmt.Sprintf("proof_generate/%d", size), func(b *testing.B) {
			scope := intervalScopeFixtureV1()
			records := make([]IntervalRecordV1, size)
			for i := range records {
				start := int64(i * 4)
				records[i] = intervalRecordFixtureV1(fmt.Sprintf("assertion:%08d", i), start, start+3)
			}
			tree, err := NewIntervalTreeV1(scope, records)
			if err != nil {
				b.Fatal(err)
			}
			query := intervalPointV1(int64(size*2), 0)
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if _, err := tree.ProvePointQueryV1(query); err != nil {
					b.Fatal(err)
				}
			}
		})
		b.Run(fmt.Sprintf("proof_verify/%d", size), func(b *testing.B) {
			scope := intervalScopeFixtureV1()
			records := make([]IntervalRecordV1, size)
			for i := range records {
				start := int64(i * 4)
				records[i] = intervalRecordFixtureV1(fmt.Sprintf("assertion:%08d", i), start, start+3)
			}
			tree, err := NewIntervalTreeV1(scope, records)
			if err != nil {
				b.Fatal(err)
			}
			query := intervalPointV1(int64(size*2), 0)
			proof, err := tree.ProvePointQueryV1(query)
			if err != nil {
				b.Fatal(err)
			}
			root := tree.RootHash()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if _, err := VerifyIntervalPointProofV1(scope, query, proof, root); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
