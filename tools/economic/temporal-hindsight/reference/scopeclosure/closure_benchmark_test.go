package scopeclosure

import (
	"fmt"
	"testing"
)

func benchmarkFixture(n int)(QueryScope,ScopeClosureManifest,UniverseSnapshot,FrontierSnapshot){
	s:=QueryScope{VerifierRef:"verifier:bench",PolicyEpochRef:"policy:bench",NamespaceRef:"ns:bench",SubjectRef:"subject:bench",ClaimID:"claim:bench",QueryProfileRef:"profile:bench"}
	digest,_:=ScopeDigest(s); candidates:=make([]OwnerCandidate,n); owners:=make([]OwnerFrontierBinding,n)
	for i:=0;i<n;i++{ref:=fmt.Sprintf("owner:%08d",i);history:=fmt.Sprintf("history:%08d:v1",i);candidates[i]=OwnerCandidate{OwnerRef:ref,HistoryRef:history,Disposition:CandidateRequired};owners[i]=OwnerFrontierBinding{OwnerRef:ref,HistoryRef:history,SequenceCut:10,TreeSize:10,LogRootDigest:"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}}
	root,_:=CandidateRoot(candidates)
	u:=UniverseSnapshot{RegistryRef:"registry:bench",RegistryEpoch:"epoch:1",ProfileRef:"universe:bench",ScopeDigest:digest,SourceProofDigest:"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",Candidates:candidates}
	m:=ScopeClosureManifest{RegistryRef:u.RegistryRef,RegistryEpoch:u.RegistryEpoch,ProfileRef:u.ProfileRef,ScopeDigest:digest,CandidateRoot:root,SourceProofDigest:u.SourceProofDigest,Candidates:candidates}
	f:=FrontierSnapshot{VerifierRef:s.VerifierRef,PolicyEpochRef:s.PolicyEpochRef,RegistryRef:u.RegistryRef,RegistryEpoch:u.RegistryEpoch,UniverseProfileRef:u.ProfileRef,ScopeDigest:digest,FrontierDigest:"cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",Owners:owners}
	return s,m,u,f
}
func BenchmarkScopeFrontierReconciliation(b *testing.B){
	for _,n:=range []int{1,10,100,1000,10000}{b.Run(fmt.Sprintf("owners=%d",n),func(b *testing.B){
		s,m,u,f:=benchmarkFixture(n);b.ReportAllocs();b.ResetTimer()
		for i:=0;i<b.N;i++{r:=Reconcile(s,m,u,f);if r.Disposition!=FrontierCoversSuppliedRoster{b.Fatal(r.Reason)}}
	})}
}
