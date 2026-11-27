package scopeclosure

import (
	"strings"
	"testing"
)

const dA = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
const dB = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

func fixture() (QueryScope, ScopeClosureManifest, UniverseSnapshot, FrontierSnapshot) {
	scope := QueryScope{VerifierRef:"verifier:1", PolicyEpochRef:"policy:7", NamespaceRef:"ns:grants", SubjectRef:"person:1", ClaimID:"grant:alpha", QueryProfileRef:"temporal-profile:v1", ValidAt:Instant{UnixSeconds:100,Nanoseconds:123}}
	candidates:=[]OwnerCandidate{{OwnerRef:"owner:A",HistoryRef:"history:A:v1",Disposition:CandidateRequired},{OwnerRef:"owner:B",HistoryRef:"history:B:v1",Disposition:CandidateRequired}}
	scopeDigest,_:=ScopeDigest(scope)
	root,_:=CandidateRoot(candidates)
	universe:=UniverseSnapshot{RegistryRef:"directory:main",RegistryEpoch:"epoch:42",ProfileRef:"owner-universe:v1",ScopeDigest:scopeDigest,SourceProofDigest:dA,Candidates:candidates}
	manifest:=ScopeClosureManifest{RegistryRef:universe.RegistryRef,RegistryEpoch:universe.RegistryEpoch,ProfileRef:universe.ProfileRef,ScopeDigest:scopeDigest,CandidateRoot:root,SourceProofDigest:dA,Candidates:candidates}
	frontier:=FrontierSnapshot{VerifierRef:scope.VerifierRef,PolicyEpochRef:scope.PolicyEpochRef,ScopeDigest:scopeDigest,FrontierDigest:dB,Owners:[]OwnerFrontierBinding{
		{OwnerRef:"owner:A",HistoryRef:"history:A:v1",SequenceCut:2,TreeSize:2,LogRootDigest:dA},
		{OwnerRef:"owner:B",HistoryRef:"history:B:v1",SequenceCut:1,TreeSize:1,LogRootDigest:dB},
	}}
	return scope,manifest,universe,frontier
}

func TestExactRosterReconcilesWithoutPromotingAuthentication(t *testing.T) {
	s,m,u,f:=fixture()
	got:=Reconcile(s,m,u,f)
	if got.Disposition!=FrontierCoversSuppliedRoster { t.Fatalf("expected structural roster reconciliation, got %+v",got) }
	if got.UniverseAuthentication!="NotEvaluated" || got.FrontierAuthentication!="NotEvaluated" { t.Fatalf("upstream trust boundary promoted: %+v",got) }
	if got.RequiredOwnerCount!=2 || got.CertificateDigest=="" { t.Fatalf("unexpected result: %+v",got) }
}

func TestValidFrontierOverOmittedOwnerIsIndeterminate(t *testing.T) {
	s,m,u,f:=fixture()
	f.Owners=f.Owners[:1]
	got:=Reconcile(s,m,u,f)
	if got.Disposition!=Indeterminate || !strings.Contains(got.Reason,"omitted") { t.Fatalf("omitted owner accepted: %+v",got) }
}

func TestUnknownWithheldAndUnavailableCandidatesAreIndeterminate(t *testing.T) {
	for _,d:=range []CandidateDisposition{CandidateUnknown,CandidateWithheld,CandidateOffline} {
		s,m,u,f:=fixture()
		u.Candidates=append(u.Candidates,OwnerCandidate{OwnerRef:"owner:C",HistoryRef:"history:C:v1",Disposition:d})
		m.Candidates=append(m.Candidates,u.Candidates[len(u.Candidates)-1])
		m.CandidateRoot,_=CandidateRoot(m.Candidates)
		got:=Reconcile(s,m,u,f)
		if got.Disposition!=Indeterminate { t.Fatalf("%s candidate must prevent closure: %+v",d,got) }
	}
}

func TestProfileExclusionMustBeExplicitAndNotInFrontier(t *testing.T) {
	s,m,u,f:=fixture()
	excluded:=OwnerCandidate{OwnerRef:"owner:C",Disposition:CandidateExcluded,ExclusionCode:"outside-namespace-v1",ExclusionProof:dB}
	m.Candidates=append(m.Candidates,excluded); u.Candidates=append(u.Candidates,excluded)
	m.CandidateRoot,_=CandidateRoot(m.Candidates)
	if got:=Reconcile(s,m,u,f); got.Disposition!=FrontierCoversSuppliedRoster { t.Fatalf("explicit exclusion failed: %+v",got) }
	f.Owners=append(f.Owners,OwnerFrontierBinding{OwnerRef:"owner:C",HistoryRef:"history:C:v1",LogRootDigest:dA})
	if got:=Reconcile(s,m,u,f); got.Disposition!=Indeterminate { t.Fatalf("excluded owner appeared in frontier: %+v",got) }
}

func TestRegistryEpochScopeAndHistorySubstitutionFailClosed(t *testing.T) {
	t.Run("registry epoch",func(t *testing.T){s,m,u,f:=fixture();u.RegistryEpoch="epoch:43";if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate{t.Fatalf("stale epoch accepted: %+v",got)}})
	t.Run("scope",func(t *testing.T){s,m,u,f:=fixture();s.ClaimID="grant:other";if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate{t.Fatalf("scope substitution accepted: %+v",got)}})
	t.Run("owner history",func(t *testing.T){s,m,u,f:=fixture();f.Owners[0].HistoryRef="history:A:forked";if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate{t.Fatalf("history substitution accepted: %+v",got)}})
}

func TestGapsAndDuplicateOwnersAreIndeterminate(t *testing.T) {
	s,m,u,f:=fixture()
	f.Owners[0].Gaps=[]GapRange{{First:1,Last:1}}
	if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate{t.Fatalf("gap accepted: %+v",got)}
	s,m,u,f=fixture();f.Owners=append(f.Owners,f.Owners[0])
	if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate{t.Fatalf("duplicate owner accepted: %+v",got)}
}

func TestEmptyRosterIsNotScopeClosure(t *testing.T) {
	s,m,u,f:=fixture()
	m.Candidates=nil;u.Candidates=nil;m.CandidateRoot,_=CandidateRoot(nil);m.SourceProofDigest=dA;u.SourceProofDigest=dA;f.Owners=nil
	got:=Reconcile(s,m,u,f)
	if got.Disposition!=FrontierCoversSuppliedRoster || got.UniverseAuthentication!="NotEvaluated" {
		t.Fatalf("empty supplied roster promoted to authenticated empty universe: %+v",got)
	}
}

func TestLaterRegistryEpochDoesNotRewriteHistoricalCertificate(t *testing.T) {
	s,m,u,f:=fixture()
	old:=Reconcile(s,m,u,f)
	u2:=u;u2.RegistryEpoch="epoch:43";u2.Candidates=append(append([]OwnerCandidate(nil),u.Candidates...),OwnerCandidate{OwnerRef:"owner:C",HistoryRef:"history:C:v1",Disposition:CandidateRequired})
	m2:=m;m2.RegistryEpoch="epoch:43";m2.Candidates=u2.Candidates;m2.CandidateRoot,_=CandidateRoot(m2.Candidates)
	current:=Reconcile(s,m2,u2,f)
	oldAgain:=Reconcile(s,m,u,f)
	if old.CertificateDigest!=oldAgain.CertificateDigest { t.Fatal("historical roster certificate changed after later epoch") }
	if current.Disposition!=Indeterminate { t.Fatalf("new owner omitted from frontier: %+v",current) }
}

func TestScopeDigestPreservesNanosecondPrecision(t *testing.T) {
	s,_,_,_:=fixture();a,_:=ScopeDigest(s);s.ValidAt.Nanoseconds++;b,_:=ScopeDigest(s)
	if a==b { t.Fatal("nanosecond change did not change scope digest") }
}

func TestDuplicateStableHistoryIdentityIsRejected(t *testing.T) {
	_,_,u,_:=fixture();u.Candidates[1].HistoryRef=u.Candidates[0].HistoryRef
	if _,err:=CandidateRoot(u.Candidates);err==nil { t.Fatal("two owners cannot share stable history identity") }
}

func TestManifestCannotSelfDeclareAnIncompleteRoster(t *testing.T) {
	s,m,u,f:=fixture();m.Candidates=m.Candidates[:1];m.CandidateRoot,_=CandidateRoot(m.Candidates)
	if got:=Reconcile(s,m,u,f);got.Disposition!=Indeterminate { t.Fatalf("manifest-substituted roster accepted: %+v",got) }
}


func TestMultipleOmissionsProduceDeterministicFailureCertificate(t *testing.T) {
	s, m, u, f := fixture()
	f.Owners = nil
	first := Reconcile(s, m, u, f)
	for i := 0; i < 100; i++ {
		got := Reconcile(s, m, u, f)
		if got.CertificateDigest != first.CertificateDigest || got.Reason != first.Reason {
			t.Fatalf("non-deterministic omission result: first=%+v got=%+v", first, got)
		}
	}
}
