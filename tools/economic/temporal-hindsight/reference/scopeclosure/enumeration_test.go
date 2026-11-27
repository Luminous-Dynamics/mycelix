package scopeclosure

import "testing"

const (
	enumA = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	enumB = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
	enumC = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
)

func enumerationFixture(t *testing.T) (EnumerationManifestV1, []EnumerationPageV1) {
	t.Helper()
	a := []OwnerCandidate{{OwnerRef:"owner:A",HistoryRef:"history:A:v1",Disposition:CandidateRequired}}
	b := []OwnerCandidate{{OwnerRef:"owner:B",HistoryRef:"history:B:v1",Disposition:CandidateRequired}}
	p0 := EnumerationPageV1{SourceRef:"directory:source",SnapshotRef:"snapshot:9",RegistryRef:"directory:main",RegistryEpoch:"epoch:9",ProfileRef:"owner-universe:v1",ScopeDigest:enumA,PredicateDigest:enumB,PageIndex:0,NextCursorDigest:enumC,Candidates:a}
	p1 := EnumerationPageV1{SourceRef:p0.SourceRef,SnapshotRef:p0.SnapshotRef,RegistryRef:p0.RegistryRef,RegistryEpoch:p0.RegistryEpoch,ProfileRef:p0.ProfileRef,ScopeDigest:p0.ScopeDigest,PredicateDigest:p0.PredicateDigest,PageIndex:1,RequestCursorDigest:enumC,Candidates:b,Terminal:true}
	var err error
	p0.PageDigest,err=EnumerationPageDigestV1(p0); if err!=nil { t.Fatal(err) }
	p1.PageDigest,err=EnumerationPageDigestV1(p1); if err!=nil { t.Fatal(err) }
	root,err:=CandidateRoot(append(append([]OwnerCandidate(nil),a...),b...)); if err!=nil { t.Fatal(err) }
	m:=EnumerationManifestV1{SourceRef:p0.SourceRef,SnapshotRef:p0.SnapshotRef,RegistryRef:p0.RegistryRef,RegistryEpoch:p0.RegistryEpoch,ProfileRef:p0.ProfileRef,ScopeDigest:p0.ScopeDigest,PredicateDigest:p0.PredicateDigest,PageCount:2,FirstPageDigest:p0.PageDigest,TerminalPageDigest:p1.PageDigest,CandidateRoot:root,SourceCompletionReceiptDigest:enumC,CompletionDisposition:SourceDeclaredComplete}
	return m,[]EnumerationPageV1{p0,p1}
}
func resealEnumerationPage(t *testing.T,p *EnumerationPageV1){t.Helper();d,e:=EnumerationPageDigestV1(*p);if e!=nil{t.Fatal(e)};p.PageDigest=d}

func TestEnumerationChainWellFormedDoesNotAuthenticateSource(t *testing.T){
	m,p:=enumerationFixture(t);r:=VerifyEnumerationPageChainV1(m,p)
	if r.Disposition!=EnumerationChainWellFormed {t.Fatalf("not well formed: %+v",r)}
	if r.PageChainAuthentication!="NotEvaluated" || r.SourceCompletenessAuthentication!="NotEvaluated" {t.Fatalf("structural check promoted trust: %+v",r)}
	if r.CertificateDigest=="" || r.PageChainDigest=="" || r.ManifestDigest=="" {t.Fatalf("missing deterministic commitments: %+v",r)}
}
func TestTruncatedEnumerationFails(t *testing.T){m,p:=enumerationFixture(t);r:=VerifyEnumerationPageChainV1(m,p[:1]);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("truncated chain accepted: %+v",r)}}
func TestPageIndexGapFails(t *testing.T){m,p:=enumerationFixture(t);p[1].PageIndex=2;resealEnumerationPage(t,&p[1]);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("page gap accepted: %+v",r)}}
func TestCursorDiscontinuityFails(t *testing.T){m,p:=enumerationFixture(t);p[1].RequestCursorDigest=enumA;resealEnumerationPage(t,&p[1]);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("cursor discontinuity accepted: %+v",r)}}
func TestCursorCycleFails(t *testing.T){m,p:=enumerationFixture(t);p[1].Terminal=false;p[1].NextCursorDigest=enumC;resealEnumerationPage(t,&p[1]);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("cursor cycle accepted: %+v",r)}}
func TestEarlyTerminalFails(t *testing.T){m,p:=enumerationFixture(t);p[0].Terminal=true;p[0].NextCursorDigest="";resealEnumerationPage(t,&p[0]);m.FirstPageDigest=p[0].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("early terminal accepted: %+v",r)}}
func TestMissingTerminalFails(t *testing.T){m,p:=enumerationFixture(t);p[1].Terminal=false;p[1].NextCursorDigest=enumA;resealEnumerationPage(t,&p[1]);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("missing terminal accepted: %+v",r)}}
func TestCrossPageDuplicateOwnerFails(t *testing.T){m,p:=enumerationFixture(t);p[1].Candidates=p[0].Candidates;resealEnumerationPage(t,&p[1]);all:=append(append([]OwnerCandidate(nil),p[0].Candidates...),p[1].Candidates...);m.CandidateRoot,_=CandidateRoot(all);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("duplicate owner across pages accepted: %+v",r)}}
func TestForeignEpochOrSnapshotOrPredicateFails(t *testing.T){
	mutations:=[]func(*EnumerationPageV1){func(p *EnumerationPageV1){p.RegistryEpoch="epoch:10"},func(p *EnumerationPageV1){p.SnapshotRef="snapshot:10"},func(p *EnumerationPageV1){p.PredicateDigest=enumC}}
	for _,mutate:=range mutations{m,p:=enumerationFixture(t);mutate(&p[1]);resealEnumerationPage(t,&p[1]);m.TerminalPageDigest=p[1].PageDigest;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("foreign page metadata accepted: %+v",r)}}
}
func TestManifestCannotDeclareCompletionWithoutReceipt(t *testing.T){m,p:=enumerationFixture(t);m.SourceCompletionReceiptDigest="";r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("completion without receipt accepted: %+v",r)}}
func TestUnknownOrIncompleteSourceStatusFails(t *testing.T){for _,status:=range []SourceCompletionDisposition{SourceCompletionUnknown,SourceDeclaredIncomplete}{m,p:=enumerationFixture(t);m.CompletionDisposition=status;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("source status %s accepted: %+v",status,r)}}}
func TestPageDigestAndAggregateRootAreBound(t *testing.T){m,p:=enumerationFixture(t);p[1].Candidates[0].HistoryRef="history:B:fork";r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("changed page data accepted: %+v",r)}}
func TestManifestCannotOmitPageDigest(t *testing.T){m,p:=enumerationFixture(t);m.TerminalPageDigest=enumA;r:=VerifyEnumerationPageChainV1(m,p);if r.Disposition!=EnumerationChainIndeterminate{t.Fatalf("wrong terminal page binding accepted: %+v",r)}}

func TestManifestCompletionEvidenceIsBoundIntoCertificate(t *testing.T){
	m,p:=enumerationFixture(t);first:=VerifyEnumerationPageChainV1(m,p)
	m.SourceCompletionReceiptDigest=enumA;second:=VerifyEnumerationPageChainV1(m,p)
	if second.Disposition!=EnumerationChainWellFormed{t.Fatalf("page chain should remain structurally valid: %+v",second)}
	if first.ManifestDigest==second.ManifestDigest || first.CertificateDigest==second.CertificateDigest{t.Fatal("completion receipt substitution did not change commitments")}
	if first.PageChainDigest!=second.PageChainDigest{t.Fatal("source completion receipt must remain separate from page-chain digest")}
}
func TestManifestDispositionIsCommittedEvenWhenIndeterminate(t *testing.T){
	m,p:=enumerationFixture(t);complete:=VerifyEnumerationPageChainV1(m,p)
	m.CompletionDisposition=SourceCompletionUnknown;unknown:=VerifyEnumerationPageChainV1(m,p)
	if unknown.Disposition!=EnumerationChainIndeterminate{t.Fatalf("unknown completion state accepted: %+v",unknown)}
	if complete.ManifestDigest==unknown.ManifestDigest || complete.CertificateDigest==unknown.CertificateDigest{t.Fatal("completion disposition not bound into certificate")}
}
func TestExpectedAndObservedPageCountsAreBound(t *testing.T){
	m,p:=enumerationFixture(t);m.PageCount=3;got:=VerifyEnumerationPageChainV1(m,p)
	if got.Disposition!=EnumerationChainIndeterminate || got.ExpectedPageCount!=3 || got.ObservedPageCount!=2{t.Fatalf("page-count mismatch evidence lost: %+v",got)}
}
func TestMalformedMetadataFailureReasonIsDeterministic(t *testing.T){
	m,p:=enumerationFixture(t);p[0].SourceRef="";p[0].SnapshotRef=""
	first:=VerifyEnumerationPageChainV1(m,p)
	for i:=0;i<100;i++{got:=VerifyEnumerationPageChainV1(m,p);if got.Reason!=first.Reason || got.CertificateDigest!=first.CertificateDigest{t.Fatalf("nondeterministic failure result: first=%+v got=%+v",first,got)}}
}
func TestCandidateOrderingWithinPageDoesNotChangeDigest(t *testing.T){
	_,p:=enumerationFixture(t);p[0].Candidates=append(p[0].Candidates,OwnerCandidate{OwnerRef:"owner:C",HistoryRef:"history:C:v1",Disposition:CandidateRequired})
	before,err:=EnumerationPageDigestV1(p[0]);if err!=nil{t.Fatal(err)}
	p[0].Candidates[0],p[0].Candidates[1]=p[0].Candidates[1],p[0].Candidates[0]
	after,err:=EnumerationPageDigestV1(p[0]);if err!=nil{t.Fatal(err)}
	if before!=after{t.Fatal("candidate ordering changed canonical page digest")}
}
