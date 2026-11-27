package progresscapability

import (
	"fmt"
	"strings"
	"testing"
)

const fixtureQueryScope = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"

func fixtureScope(owners ...string) CapabilityScopeV1 {
	return CapabilityScopeV1{SourceRef:"source:directory",RegistryRef:"registry:main",RegistryEpoch:"epoch:4",ProfileRef:"owners:v1",QueryScopeDigest:fixtureQueryScope,OwnerRefs:owners}
}
func fixtureTransition(kind CapabilityTransitionKindV1,id string,generation uint64,prior string,cut map[string]uint64) CapabilityTransitionV1 {
	coords:=make([]OwnerCoordinateV1,0,len(cut))
	for owner,seq:=range cut { coords=append(coords,OwnerCoordinateV1{OwnerRef:owner,Sequence:seq}) }
	return CapabilityTransitionV1{Kind:kind,CapabilityID:id,Generation:generation,PriorCapabilityDigest:prior,OwnerCut:coords,
		IssuerRef:"issuer:1",IssuerKeyEpoch:"key:1",AuthorityGrantRef:"grant:1",CheckpointRef:"checkpoint:1"}
}
func fixtureGenesis(t *testing.T,scope CapabilityScopeV1)string{
	t.Helper();digest,err:=ScopeDigestV1(scope);if err!=nil{t.Fatal(err)};return genesisDigest(digest)
}
func fixtureAppend(t *testing.T,scope CapabilityScopeV1,events []CapabilityEventV1,id string,outcome CapabilityOutcomeV1,transition *CapabilityTransitionV1,reconciles string)[]CapabilityEventV1{
	t.Helper();previous:=fixtureGenesis(t,scope);if len(events)>0{previous=events[len(events)-1].EventDigest}
	scopeDigest,err:=ScopeDigestV1(scope);if err!=nil{t.Fatal(err)}
	event:=CapabilityEventV1{LogSequence:uint64(len(events)+1),EventID:id,PreviousEventDigest:previous,Outcome:outcome,Transition:transition,ReconcilesEventID:reconciles}
	event.EventDigest,err=EventDigestV1(scopeDigest,event);if err!=nil{t.Fatal(err)}
	return append(events,event)
}
func fixtureReplay(t *testing.T,scope CapabilityScopeV1,events []CapabilityEventV1)ReplayResultV1{
	t.Helper();tail:=fixtureGenesis(t,scope);if len(events)>0{tail=events[len(events)-1].EventDigest}
	return ReplayCapabilityLedgerV1(scope,events,uint64(len(events)),tail)
}

func TestMintAdvanceReleaseLifecycleDerivesEmptyDeclaredProgress(t *testing.T){
	scope:=fixtureScope("owner:A","owner:B")
	mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":2,"owner:B":1})
	events:=fixtureAppend(t,scope,nil,"event:mint",OutcomeCommitted,&mint,"")
	minted:=fixtureReplay(t,scope,events)
	if minted.Disposition!=LedgerValid||len(minted.ActiveCapabilities)!=1{t.Fatalf("mint replay: %+v",minted)}
	advance:=fixtureTransition(TransitionAdvance,"cap:1",2,minted.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":3,"owner:B":1})
	events=fixtureAppend(t,scope,events,"event:advance",OutcomeCommitted,&advance,"")
	advanced:=fixtureReplay(t,scope,events)
	if advanced.Disposition!=LedgerValid||advanced.ActiveCapabilities[0].Generation!=2{t.Fatalf("advance replay: %+v",advanced)}
	release:=fixtureTransition(TransitionRelease,"cap:1",3,advanced.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":3,"owner:B":1})
	events=fixtureAppend(t,scope,events,"event:release",OutcomeCommitted,&release,"")
	released:=fixtureReplay(t,scope,events)
	if released.Disposition!=LedgerValid||len(released.ActiveCapabilities)!=0{t.Fatalf("release replay: %+v",released)}
	progress:=DeriveProgressAntichainV1(scope,released)
	if progress.Disposition!=ProgressDeclaredClosed||len(progress.ProgressPoints)!=0{t.Fatalf("empty progress not derived: %+v",progress)}
	if progress.ProgressAuthentication!=NotEvaluated||progress.SourceUniverseCompleteness!=NotEvaluated{t.Fatalf("declared closure promoted trust: %+v",progress)}
}

func TestUnknownReleaseBlocksMutationAndProgressUntilReconciled(t *testing.T){
	scope:=fixtureScope("owner:A")
	mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":4})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	state:=fixtureReplay(t,scope,events)
	release:=fixtureTransition(TransitionRelease,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":4})
	events=fixtureAppend(t,scope,events,"release-unknown",OutcomeUnknown,&release,"")
	blocked:=fixtureReplay(t,scope,events)
	if !blocked.MutationBlocked||blocked.PendingEventID!="release-unknown"||len(blocked.ActiveCapabilities)!=1{t.Fatalf("unknown release not held: %+v",blocked)}
	if got:=DeriveProgressAntichainV1(scope,blocked);got.Disposition!=ProgressUnavailable{t.Fatalf("unknown release advanced progress: %+v",got)}
	advance:=fixtureTransition(TransitionAdvance,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":5})
	illegal:=fixtureAppend(t,scope,events,"illegal-advance",OutcomeCommitted,&advance,"")
	if got:=fixtureReplay(t,scope,illegal);got.Disposition!=LedgerIndeterminate{t.Fatalf("mutation during unknown outcome accepted: %+v",got)}
	events=fixtureAppend(t,scope,events,"reconcile-not-applied",OutcomeNotApplied,nil,"release-unknown")
	reconciled:=fixtureReplay(t,scope,events)
	if reconciled.Disposition!=LedgerValid||reconciled.MutationBlocked||len(reconciled.ActiveCapabilities)!=1{t.Fatalf("not-applied reconcile failed: %+v",reconciled)}
}

func TestReconciledAppliedReleaseConsumesCapability(t *testing.T){
	scope:=fixtureScope("owner:A")
	mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":2})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	state:=fixtureReplay(t,scope,events)
	release:=fixtureTransition(TransitionRelease,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":2})
	events=fixtureAppend(t,scope,events,"unknown-release",OutcomeUnknown,&release,"")
	events=fixtureAppend(t,scope,events,"reconcile-applied",OutcomeApplied,nil,"unknown-release")
	got:=fixtureReplay(t,scope,events)
	if got.Disposition!=LedgerValid||len(got.ActiveCapabilities)!=0{t.Fatalf("applied reconciliation did not release: %+v",got)}
}

func TestUnknownMintOnlyActivatesOnAppliedReconciliation(t *testing.T){
	scope:=fixtureScope("owner:A")
	mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":2})
	events:=fixtureAppend(t,scope,nil,"unknown-mint",OutcomeUnknown,&mint,"")
	pending:=fixtureReplay(t,scope,events)
	if len(pending.ActiveCapabilities)!=0||!pending.MutationBlocked{t.Fatalf("unknown mint took effect early: %+v",pending)}
	events=fixtureAppend(t,scope,events,"not-applied",OutcomeNotApplied,nil,"unknown-mint")
	if got:=fixtureReplay(t,scope,events);len(got.ActiveCapabilities)!=0{t.Fatalf("not-applied mint created a capability: %+v",got)}
	events=events[:1]
	events=fixtureAppend(t,scope,events,"applied",OutcomeApplied,nil,"unknown-mint")
	if got:=fixtureReplay(t,scope,events);len(got.ActiveCapabilities)!=1{t.Fatalf("applied mint was not committed: %+v",got)}
}

func TestUnknownAdvanceLeavesOldCapabilityCutUntilReconciled(t *testing.T){
	scope:=fixtureScope("owner:A")
	mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	state:=fixtureReplay(t,scope,events)
	advance:=fixtureTransition(TransitionAdvance,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":2})
	events=fixtureAppend(t,scope,events,"unknown-advance",OutcomeUnknown,&advance,"")
	got:=fixtureReplay(t,scope,events)
	if !got.MutationBlocked||len(got.ActiveCapabilities)!=1||got.ActiveCapabilities[0].OwnerCut[0].Sequence!=1{t.Fatalf("unknown advance changed active state: %+v",got)}
}

func TestCapabilityIDsCannotBeReusedAfterReleaseOrRevoke(t *testing.T){
	for _,terminal:=range []CapabilityTransitionKindV1{TransitionRelease,TransitionRevoke}{
		t.Run(string(terminal),func(t *testing.T){
			scope:=fixtureScope("owner:A")
			mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":1})
			events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
			state:=fixtureReplay(t,scope,events)
			terminalEvent:=fixtureTransition(terminal,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":1})
			events=fixtureAppend(t,scope,events,"terminal",OutcomeCommitted,&terminalEvent,"")
			reuse:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":2})
			events=fixtureAppend(t,scope,events,"reuse",OutcomeCommitted,&reuse,"")
			if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatalf("reused capability ID accepted: %+v",got)}
		})
	}
}

func TestDuplicateReleaseIsRejected(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");state:=fixtureReplay(t,scope,events)
	release:=fixtureTransition(TransitionRelease,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":1})
	events=fixtureAppend(t,scope,events,"release",OutcomeCommitted,&release,"")
	events=fixtureAppend(t,scope,events,"release-again",OutcomeCommitted,&release,"")
	if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatal("duplicate release accepted")}
}

func TestInvalidGenerationPredecessorAndCutRegressionFail(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":5})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");state:=fixtureReplay(t,scope,events)
	cases:=[]struct{name string;transition CapabilityTransitionV1}{
		{"generation",fixtureTransition(TransitionAdvance,"cap:1",3,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":6})},
		{"predecessor",fixtureTransition(TransitionAdvance,"cap:1",2,fixtureQueryScope,map[string]uint64{"owner:A":6})},
		{"regression",fixtureTransition(TransitionAdvance,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":4})},
	}
	for _,test:=range cases{t.Run(test.name,func(t *testing.T){events:=fixtureAppend(t,scope,events,"bad:"+test.name,OutcomeCommitted,&test.transition,"");if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatalf("invalid transition accepted: %+v",got)}})}
}

func TestReleaseCannotRewriteLowerBoundCut(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":5})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");state:=fixtureReplay(t,scope,events)
	release:=fixtureTransition(TransitionRelease,"cap:1",2,state.ActiveCapabilities[0].TransitionDigest,map[string]uint64{"owner:A":6})
	events=fixtureAppend(t,scope,events,"bad-release",OutcomeCommitted,&release,"")
	if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatal("release rewrote lower-bound cut")}
}

func TestEventSequenceTailAndTamperingFailClosed(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	if got:=ReplayCapabilityLedgerV1(scope,events,2,events[0].EventDigest);got.Disposition!=LedgerIndeterminate{t.Fatal("wrong event count accepted")}
	if got:=ReplayCapabilityLedgerV1(scope,events,1,fixtureQueryScope);got.Disposition!=LedgerIndeterminate{t.Fatal("wrong tail anchor accepted")}
	tampered:=append([]CapabilityEventV1(nil),events...);copyTransition:=cloneTransition(*tampered[0].Transition);copyTransition.CapabilityID="cap:other";tampered[0].Transition=&copyTransition
	if got:=fixtureReplay(t,scope,tampered);got.Disposition!=LedgerIndeterminate{t.Fatal("tampered transition accepted")}
	duplicateID:=fixtureAppend(t,scope,events,"mint",OutcomeCommitted,&mint,"")
	if got:=fixtureReplay(t,scope,duplicateID);got.Disposition!=LedgerIndeterminate{t.Fatal("duplicate event ID accepted")}
}

func TestUnknownOutcomeMustBeReconciledByExactEvent(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"unknown",OutcomeUnknown,&mint,"")
	events=fixtureAppend(t,scope,events,"wrong-reconcile",OutcomeApplied,nil,"other")
	if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatal("wrong pending event reconciliation accepted")}
}

func TestUnresolvedUnknownAtTailBlocksProgressDerivation(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:1",1,"",map[string]uint64{"owner:A":2})
	events:=fixtureAppend(t,scope,nil,"unknown",OutcomeUnknown,&mint,"")
	got:=fixtureReplay(t,scope,events)
	if !got.MutationBlocked||got.Disposition!=LedgerIndeterminate{t.Fatalf("unknown outcome not blocked: %+v",got)}
	if progress:=DeriveProgressAntichainV1(scope,got);progress.Disposition!=ProgressUnavailable{t.Fatalf("pending outcome allowed progress derivation: %+v",progress)}
}

func TestDerivedProgressNormalizesDominatedAndKeepsIncomparableCuts(t *testing.T){
	scope:=fixtureScope("owner:A","owner:B")
	a:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":1,"owner:B":3})
	events:=fixtureAppend(t,scope,nil,"mint:A",OutcomeCommitted,&a,"")
	b:=fixtureTransition(TransitionMint,"cap:B",1,"",map[string]uint64{"owner:A":2,"owner:B":4})
	events=fixtureAppend(t,scope,events,"mint:B",OutcomeCommitted,&b,"")
	c:=fixtureTransition(TransitionMint,"cap:C",1,"",map[string]uint64{"owner:A":3,"owner:B":1})
	events=fixtureAppend(t,scope,events,"mint:C",OutcomeCommitted,&c,"")
	got:=fixtureReplay(t,scope,events)
	progress:=DeriveProgressAntichainV1(scope,got)
	if progress.Disposition!=ProgressDerived||len(progress.ProgressPoints)!=2{t.Fatalf("expected minimal antichain of width two: %+v",progress)}
}

func TestScopeOrEpochSubstitutionBlocksDerivation(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");got:=fixtureReplay(t,scope,events)
	other:=scope;other.RegistryEpoch="epoch:5"
	if progress:=DeriveProgressAntichainV1(other,got);progress.Disposition!=ProgressUnavailable{t.Fatal("scope/epoch substitution accepted")}
}

func TestEmptyOwnerUniverseCanBeExplicitlyBound(t *testing.T){
	scope:=fixtureScope()
	if _,err:=ScopeDigestV1(scope);err!=nil{t.Fatalf("explicit empty profile scope rejected: %v",err)}
}

func TestMissingAuthorityMetadataRejected(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":1});mint.AuthorityGrantRef=""
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	if got:=fixtureReplay(t,scope,events);got.Disposition!=LedgerIndeterminate{t.Fatal("missing authority grant reference accepted")}
}


func TestEarlyCountAndTailFailuresBindSubmittedEventStream(t *testing.T) {
	scope := fixtureScope("owner:A")
	mintA := fixtureTransition(TransitionMint, "cap:A", 1, "", map[string]uint64{"owner:A": 1})
	mintB := fixtureTransition(TransitionMint, "cap:B", 1, "", map[string]uint64{"owner:A": 1})
	eventsA := fixtureAppend(t, scope, nil, "event:A", OutcomeCommitted, &mintA, "")
	eventsB := fixtureAppend(t, scope, nil, "event:B", OutcomeCommitted, &mintB, "")
	goodTail := eventsA[0].EventDigest

	countA := ReplayCapabilityLedgerV1(scope, eventsA, 2, goodTail)
	countB := ReplayCapabilityLedgerV1(scope, eventsB, 2, goodTail)
	if countA.Disposition != LedgerIndeterminate || countB.Disposition != LedgerIndeterminate {
		t.Fatal("event count mismatch unexpectedly passed")
	}
	if countA.SubmittedEventsDigest == countB.SubmittedEventsDigest || countA.CertificateDigest == countB.CertificateDigest {
		t.Fatal("count-mismatch receipts did not bind their submitted event streams")
	}

	tailA := ReplayCapabilityLedgerV1(scope, eventsA, 1, fixtureQueryScope)
	tailB := ReplayCapabilityLedgerV1(scope, eventsB, 1, fixtureQueryScope)
	if tailA.SubmittedEventsDigest == tailB.SubmittedEventsDigest || tailA.CertificateDigest == tailB.CertificateDigest {
		t.Fatal("invalid-tail receipts did not bind their submitted event streams")
	}
}

func TestInvalidScopeFailureStillBindsSubmittedEvents(t *testing.T) {
	goodScope := fixtureScope("owner:A")
	mint := fixtureTransition(TransitionMint, "cap:A", 1, "", map[string]uint64{"owner:A": 1})
	events := fixtureAppend(t, goodScope, nil, "event:A", OutcomeCommitted, &mint, "")
	badScope := goodScope
	badScope.RegistryRef = ""
	got := ReplayCapabilityLedgerV1(badScope, events, 1, events[0].EventDigest)
	if got.Disposition != LedgerIndeterminate || got.SubmittedEventsDigest == "" {
		t.Fatalf("invalid scope did not retain bounded event submission commitment: %+v", got)
	}
}

func TestLedgerEncodingBudgetRejectsBeforeRawHashing(t *testing.T) {
	scope := fixtureScope("owner:A")
	mint := fixtureTransition(TransitionMint, "cap:A", 1, "", map[string]uint64{"owner:A": 1})
	events := fixtureAppend(t, scope, nil, "event:A", OutcomeCommitted, &mint, "")
	events[0].EventID = strings.Repeat("x", MaxIdentifierBytes+1)
	got := ReplayCapabilityLedgerV1(scope, events, 1, events[0].EventDigest)
	if got.Disposition != LedgerIndeterminate || got.Reason != "ledger event encoding byte budget exceeded" {
		t.Fatalf("oversized event did not fail at bounded preflight: %+v", got)
	}
	if got.SubmittedEventsDigest != "" {
		t.Fatal("oversized event should not be fully raw-hashed before preflight rejects it")
	}
}

func TestDistinctMalformedSubmissionsHaveDistinctReceipts(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"")
	a:=append([]CapabilityEventV1(nil),events...);b:=append([]CapabilityEventV1(nil),events...)
	a[0].Transition= nil;b[0].EventID="different"
	first:=ReplayCapabilityLedgerV1(scope,a,1,events[0].EventDigest)
	second:=ReplayCapabilityLedgerV1(scope,b,1,events[0].EventDigest)
	if first.Disposition!=LedgerIndeterminate||second.Disposition!=LedgerIndeterminate{t.Fatal("malformed submissions unexpectedly passed")}
	if first.SubmittedEventsDigest==second.SubmittedEventsDigest||first.CertificateDigest==second.CertificateDigest{t.Fatal("different malformed submissions collapsed to same receipt")}
}

func TestAuthenticationAndCompletenessNeverPromoted(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":1})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");replay:=fixtureReplay(t,scope,events)
	if replay.LedgerAuthentication!=NotEvaluated||replay.IssuerAuthorityAuthentication!=NotEvaluated||replay.CheckpointAuthentication!=NotEvaluated||replay.SourceUniverseCompleteness!=NotEvaluated{t.Fatalf("structural ledger promoted trust: %+v",replay)}
	progress:=DeriveProgressAntichainV1(scope,replay)
	if progress.ProgressAuthentication!=NotEvaluated||progress.SourceUniverseCompleteness!=NotEvaluated{t.Fatalf("derived progress promoted trust: %+v",progress)}
}

func TestReplayCertificateDeterministic(t *testing.T){
	scope:=fixtureScope("owner:A");mint:=fixtureTransition(TransitionMint,"cap:A",1,"",map[string]uint64{"owner:A":2})
	events:=fixtureAppend(t,scope,nil,"mint",OutcomeCommitted,&mint,"");first:=fixtureReplay(t,scope,events)
	for i:=0;i<100;i++{next:=fixtureReplay(t,scope,events);if first.CertificateDigest!=next.CertificateDigest{t.Fatal("replay receipt is nondeterministic")}}
}

func BenchmarkCapabilityLedgerReplay(b *testing.B){
	for _,count:=range []int{1,10,100,1000,10000}{
		b.Run(fmt.Sprintf("events=%d",count),func(b *testing.B){
			scope:=fixtureScope("owner:A")
			scopeDigest,_:=ScopeDigestV1(scope)
			events:=make([]CapabilityEventV1,0,count);previous:=genesisDigest(scopeDigest)
			for i:=0;i<count;i++{
				tr:=fixtureTransition(TransitionMint,fmt.Sprintf("cap:%d",i),1,"",map[string]uint64{"owner:A":uint64(i)})
				event:=CapabilityEventV1{LogSequence:uint64(i+1),EventID:fmt.Sprintf("event:%d",i),PreviousEventDigest:previous,Outcome:OutcomeCommitted,Transition:&tr}
				event.EventDigest,_=EventDigestV1(scopeDigest,event);events=append(events,event);previous=event.EventDigest
			}
			b.ReportAllocs();b.ResetTimer()
			for i:=0;i<b.N;i++{got:=ReplayCapabilityLedgerV1(scope,events,uint64(count),previous);if got.Disposition!=LedgerValid{b.Fatalf("replay failed: %s",got.Reason)}}
		})
	}
}
