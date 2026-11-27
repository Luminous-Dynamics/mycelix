package hindsight

import (
	"math/rand"
	"reflect"
	"testing"
)

func fixture() (Scope, []string, map[string][]Event, Cut) {
	scope := Scope{Verifier: "verifier:1", Policy: "policy:7", Namespace: "ns:grants", Subject: "person:1", Claim: "grant:alpha"}
	owners := []string{"owner:A", "owner:B"}
	h := map[string][]Event{
		"owner:A": {
			{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 1, ID: "A-start", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:A:1", Dimension: ValidFrom, Value: 0},
			{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 2, ID: "A-open", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:A:1", Dimension: ValidTo, ExplicitOpen: true},
		},
		"owner:B": {
			{Owner: "owner:B", OwnerHistoryRef: "history:B:v1", Namespace: "ns:grants", Sequence: 1, ID: "B-start", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:B:1", Dimension: ValidFrom, Value: 0},
			{Owner: "owner:B", OwnerHistoryRef: "history:B:v1", Namespace: "ns:grants", Sequence: 2, ID: "B-open", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:B:1", Dimension: ValidTo, ExplicitOpen: true},
		},
	}
	return scope, owners, h, Cut{"owner:A": {OwnerHistoryRef: "history:A:v1", Sequence: 2}, "owner:B": {OwnerHistoryRef: "history:B:v1", Sequence: 2}}
}

func appendCorrection(h map[string][]Event, owner, ref, prior, id string, sequence uint64, end int64) {
	h[owner] = append(h[owner], Event{Owner: owner, OwnerHistoryRef: map[string]string{"owner:A": "history:A:v1", "owner:B": "history:B:v1"}[owner], Namespace: "ns:grants", Sequence: sequence, ID: id, Subject: "person:1", Claim: "grant:alpha", IntervalRef: ref, Dimension: ValidTo, Value: end, Supersedes: prior})
}

func TestHalfOpenAndExplicitOpen(t *testing.T) {
	scope, owners, h, cut := fixture()
	atStart := Evaluate(owners, h, cut, scope, 0)
	if atStart.Disposition != "CompleteForSuppliedOwnerPrefixes" || len(atStart.Matches) != 2 {
		t.Fatalf("start must be inclusive: %+v", atStart)
	}
	appendCorrection(h, "owner:A", "episode:A:1", "A-open", "A-close", 3, 10)
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 3}
	atEnd := Evaluate(owners, h, cut, scope, 10)
	if len(atEnd.Matches) != 1 || atEnd.Matches[0].Owner != "owner:B" {
		t.Fatalf("finite end must be exclusive: %+v", atEnd)
	}
	atBeforeEnd := Evaluate(owners, h, cut, scope, 9)
	if len(atBeforeEnd.Matches) != 2 {
		t.Fatalf("point before finite end should match: %+v", atBeforeEnd)
	}
}

func TestBackdatedSuffixCannotChangeHistoricalReplay(t *testing.T) {
	scope, owners, h, f0 := fixture()
	base := Evaluate(owners, h, f0, scope, 10)
	if len(base.Matches) != 2 {
		t.Fatalf("expected both explicitly-open intervals: %+v", base)
	}
	appendCorrection(h, "owner:A", "episode:A:1", "A-open", "A-backdated-revocation", 3, 5)
	appendCorrection(h, "owner:B", "episode:B:1", "B-open", "B-backdated-revocation", 3, 5)
	replica1 := map[string][]Event{"owner:A": append([]Event(nil), h["owner:A"]...), "owner:B": append([]Event(nil), h["owner:B"]...)}
	replica2 := map[string][]Event{"owner:A": append([]Event(nil), h["owner:A"][:2]...), "owner:B": append([]Event(nil), h["owner:B"][:2]...)}
	r1 := Evaluate(owners, replica1, f0, scope, 10)
	r2 := Evaluate(owners, replica2, f0, scope, 10)
	if !reflect.DeepEqual(r1.Matches, r2.Matches) || r1.Certificate != r2.Certificate || base.Certificate != r1.Certificate {
		t.Fatalf("suffix changed historical F0 replay: base=%+v r1=%+v r2=%+v", base, r1, r2)
	}
	f1 := Cut{"owner:A": {OwnerHistoryRef: "history:A:v1", Sequence: 3}, "owner:B": {OwnerHistoryRef: "history:B:v1", Sequence: 3}}
	current := Evaluate(owners, replica1, f1, scope, 10)
	if len(current.Matches) != 0 {
		t.Fatalf("later accepted backdated corrections must change current answer: %+v", current)
	}
	if base.Certificate == current.Certificate {
		t.Fatal("historical/current certificates must bind different frontiers/results")
	}
}

func TestValidityEpisodeIdentityPreventsCrossPairing(t *testing.T) {
	scope := Scope{Verifier: "verifier:1", Policy: "policy:7", Namespace: "ns:grants", Subject: "person:1", Claim: "grant:alpha"}
	owners := []string{"owner:A"}
	h := map[string][]Event{"owner:A": {
		{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 1, ID: "ep1-start", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:1", Dimension: ValidFrom, Value: 0},
		{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 2, ID: "ep1-end", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:1", Dimension: ValidTo, Value: 10},
		{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 3, ID: "ep2-start", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:2", Dimension: ValidFrom, Value: 20},
		{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 4, ID: "ep2-end", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:2", Dimension: ValidTo, Value: 30},
	}}
	cut := Cut{"owner:A": {OwnerHistoryRef: "history:A:v1", Sequence: 4}}
	r := Evaluate(owners, h, cut, scope, 15)
	if r.Disposition != "CompleteForSuppliedOwnerPrefixes" || len(r.Matches) != 0 {
		t.Fatalf("cross-episode endpoint pairing manufactured a match: %+v", r)
	}
	r = Evaluate(owners, h, cut, scope, 25)
	if len(r.Matches) != 1 || r.Matches[0].IntervalRef != "episode:2" {
		t.Fatalf("second episode did not resolve independently: %+v", r)
	}
}

func TestMissingEndpointIsNotImplicitOpen(t *testing.T) {
	scope, owners, h, cut := fixture()
	h["owner:A"] = h["owner:A"][:1]
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 1}
	r := Evaluate(owners, h, cut, scope, 10)
	if r.Disposition != "Indeterminate" {
		t.Fatalf("missing VALID_TO must be indeterminate, got %+v", r)
	}
}

func TestWrongEpisodeSupersessionIsIndeterminate(t *testing.T) {
	scope, owners, h, cut := fixture()
	h["owner:A"] = append(h["owner:A"], Event{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 3, ID: "bad-correction", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:A:2", Dimension: ValidTo, Value: 5, Supersedes: "A-open"})
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 3}
	r := Evaluate(owners, h, cut, scope, 10)
	if r.Disposition != "Indeterminate" {
		t.Fatalf("cross-episode supersession must fail closed: %+v", r)
	}
}

func TestMissingOwnerAndPrefixGapAreIndeterminate(t *testing.T) {
	scope, owners, h, cut := fixture()
	delete(h, "owner:B")
	if got := Evaluate(owners, h, cut, scope, 10); got.Disposition != "Indeterminate" {
		t.Fatalf("missing owner accepted: %+v", got)
	}
	_, owners, h, cut = fixture()
	h["owner:A"] = h["owner:A"][1:]
	if got := Evaluate(owners, h, cut, scope, 10); got.Disposition != "Indeterminate" {
		t.Fatalf("prefix gap accepted: %+v", got)
	}
}

func TestOwnerInterleavingDoesNotChangeResult(t *testing.T) {
	scope, owners, h, cut := fixture()
	appendCorrection(h, "owner:A", "episode:A:1", "A-open", "A-close", 3, 7)
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 3}
	want := Evaluate(owners, h, cut, scope, 10)
	for seed := int64(0); seed < 100; seed++ {
		rng := rand.New(rand.NewSource(seed))
		copyHist := map[string][]Event{}
		for owner, events := range h {
			xs := append([]Event(nil), events...)
			rng.Shuffle(len(xs), func(i, j int) { xs[i], xs[j] = xs[j], xs[i] })
			copyHist[owner] = xs
		}
		got := Evaluate(owners, copyHist, cut, scope, 10)
		if !reflect.DeepEqual(got.Matches, want.Matches) || got.Certificate != want.Certificate {
			t.Fatalf("interleaving changed result at seed %d: %+v vs %+v", seed, got, want)
		}
	}
}

func TestLatestStateMutantProducesHindsightCounterexample(t *testing.T) {
	scope, owners, h, f0 := fixture()
	appendCorrection(h, "owner:A", "episode:A:1", "A-open", "A-revoked", 3, 5)
	correct := Evaluate(owners, h, f0, scope, 10)
	mutant := LatestStateMutant(owners, h, f0, scope, 10)
	if len(correct.Matches) != 2 {
		t.Fatalf("correct F0 replay changed: %+v", correct)
	}
	if len(mutant.Matches) != 1 {
		t.Fatalf("expected mutant to expose hindsight leak, got %+v", mutant)
	}
}

func TestSuffixNoninterferenceAcrossDeterministicMutations(t *testing.T) {
	scope, owners, h, f0 := fixture()
	baseline := Evaluate(owners, h, f0, scope, 10)
	for i := uint64(0); i < 10000; i++ {
		copyHist := map[string][]Event{"owner:A": append([]Event(nil), h["owner:A"]...), "owner:B": append([]Event(nil), h["owner:B"]...)}
		owner := owners[i%uint64(len(owners))]
		ref, prior := "episode:A:1", "A-open"
		if owner == "owner:B" {
			ref, prior = "episode:B:1", "B-open"
		}
		copyHist[owner] = append(copyHist[owner], Event{Owner: owner, OwnerHistoryRef: map[string]string{"owner:A": "history:A:v1", "owner:B": "history:B:v1"}[owner], Namespace: "ns:grants", Sequence: 3, ID: "suffix-" + owner, Subject: "person:1", Claim: "grant:alpha", IntervalRef: ref, Dimension: ValidTo, Value: int64(i % 11), Supersedes: prior})
		got := Evaluate(owners, copyHist, f0, scope, 10)
		if !reflect.DeepEqual(got.Matches, baseline.Matches) || got.Certificate != baseline.Certificate {
			t.Fatalf("suffix-only mutation %d changed F0 replay", i)
		}
	}
}

func TestAcceptedOwnerHistoryIdentityIsBound(t *testing.T) {
	scope, owners, h, cut := fixture()
	h["owner:A"][0].OwnerHistoryRef = "history:A:forked"
	r := Evaluate(owners, h, cut, scope, 10)
	if r.Disposition != "Indeterminate" {
		t.Fatalf("history identity substitution in accepted prefix must fail closed: %+v", r)
	}
}

func TestMalformedUnobservedSuffixCannotChangeHistoricalReplay(t *testing.T) {
	scope, owners, h, cut := fixture()
	base := Evaluate(owners, h, cut, scope, 10)
	h["owner:A"] = append(h["owner:A"], Event{Owner: "attacker", OwnerHistoryRef: "fork", Sequence: 999, ID: "not-yet-accepted"})
	got := Evaluate(owners, h, cut, scope, 10)
	if got.Disposition != base.Disposition || got.Certificate != base.Certificate {
		t.Fatalf("unobserved suffix affected F0: base=%+v got=%+v", base, got)
	}
}

func TestConflictingEndpointHeadsAreIndeterminate(t *testing.T) {
	scope, owners, h, cut := fixture()
	h["owner:A"] = append(h["owner:A"], Event{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:grants", Sequence: 3, ID: "A-open-conflict", Subject: "person:1", Claim: "grant:alpha", IntervalRef: "episode:A:1", Dimension: ValidTo, ExplicitOpen: true})
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 3}
	r := Evaluate(owners, h, cut, scope, 10)
	if r.Disposition != "Indeterminate" {
		t.Fatalf("competing endpoint heads must not use last-writer-wins: %+v", r)
	}
}

func TestCertificateBindsOwnerHistoryIdentityAndFrontier(t *testing.T) {
	scope, owners, h, f0 := fixture()
	r0 := Evaluate(owners, h, f0, scope, 10)
	f1 := Cut{"owner:A": {OwnerHistoryRef: "history:A:v1", Sequence: 2}, "owner:B": {OwnerHistoryRef: "history:B:v1", Sequence: 1}}
	r1 := Evaluate(owners, h, f1, scope, 10)
	if r0.Certificate == r1.Certificate {
		t.Fatal("different owner-vector frontiers must not share a certificate digest")
	}
}

func TestGoldenCertificateVector(t *testing.T) {
	scope, owners, h, cut := fixture()
	r := Evaluate(owners, h, cut, scope, 10)
	const wantCertificate = "56a1a47b23e520d8820c386f407a640cec361b58e3b79f7a7d1e8adc0eab3047"
	if r.Certificate != wantCertificate || len(r.Matches) != 2 || r.Disposition != "CompleteForSuppliedOwnerPrefixes" {
		t.Fatalf("canonical result vector mismatch: certificate=%s matches=%d disposition=%s", r.Certificate, len(r.Matches), r.Disposition)
	}
}

func TestOutOfScopeIncompleteEpisodeDoesNotContaminateQuery(t *testing.T) {
	scope, owners, h, cut := fixture()
	h["owner:A"] = append(h["owner:A"], Event{Owner: "owner:A", OwnerHistoryRef: "history:A:v1", Namespace: "ns:other", Sequence: 3, ID: "other-scope-start", Subject: "person:1", Claim: "grant:other", IntervalRef: "other-episode", Dimension: ValidFrom, Value: 100})
	cut["owner:A"] = OwnerCut{OwnerHistoryRef: "history:A:v1", Sequence: 3}
	r := Evaluate(owners, h, cut, scope, 10)
	if r.Disposition != "CompleteForSuppliedOwnerPrefixes" || len(r.Matches) != 2 {
		t.Fatalf("out-of-scope incomplete episode contaminated scoped query: %+v", r)
	}
}
