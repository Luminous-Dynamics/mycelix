package scopeclosure

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
	"strings"
)

type Instant struct { UnixSeconds int64; Nanoseconds uint32 }
type QueryScope struct {
	VerifierRef, PolicyEpochRef, NamespaceRef, SubjectRef, ClaimID, QueryProfileRef string
	ValidAt Instant
}
type CandidateDisposition string
const (
	CandidateRequired CandidateDisposition = "Required"
	CandidateExcluded CandidateDisposition = "ExcludedByDeclaredProfile"
	CandidateUnknown CandidateDisposition = "Unknown"
	CandidateWithheld CandidateDisposition = "Withheld"
	CandidateOffline CandidateDisposition = "Unavailable"
)
type OwnerCandidate struct {
	OwnerRef, HistoryRef string
	Disposition CandidateDisposition
	ExclusionCode, ExclusionProof string
}
type UniverseSnapshot struct {
	RegistryRef, RegistryEpoch, ProfileRef, ScopeDigest, SourceProofDigest string
	Candidates []OwnerCandidate
}
type ScopeClosureManifest struct {
	RegistryRef, RegistryEpoch, ProfileRef, ScopeDigest, CandidateRoot, SourceProofDigest string
	Candidates []OwnerCandidate
}
type GapRange struct { First, Last uint64 }
type OwnerFrontierBinding struct {
	OwnerRef, HistoryRef string
	SequenceCut, TreeSize uint64
	LogRootDigest string
	Gaps []GapRange
}
type FrontierSnapshot struct {
	VerifierRef, PolicyEpochRef, ScopeDigest, FrontierDigest string
	Owners []OwnerFrontierBinding
}
type Disposition string
const (
	RosterReconcilesSuppliedUniverse Disposition = "RosterReconcilesSuppliedExpectedUniverse"
	FrontierCoversSuppliedRoster Disposition = "FrontierCoversSuppliedRequiredRoster"
	Indeterminate Disposition = "Indeterminate"
)
type Result struct {
	Disposition Disposition
	Reason, ScopeDigest, CandidateRoot, FrontierDigest string
	RegistryRef, RegistryEpoch, UniverseProfile, SourceProofDigest string
	RequiredOwnerCount, ExcludedOwnerCount int
	UniverseAuthentication, FrontierAuthentication string
	CertificateDigest string
}
var ErrIndeterminate = errors.New("scope-to-frontier reconciliation is indeterminate")

// ScopeDigest commits the exact query identity and nanosecond-precision valid-time instant.
// It is a deterministic commitment, not a signature or authority assertion.
func ScopeDigest(s QueryScope) (string, error) {
	if s.ValidAt.Nanoseconds >= 1_000_000_000 { return "", errors.New("nanoseconds out of range") }
	fields := []struct{name, value string}{
		{"verifier",s.VerifierRef},{"policy epoch",s.PolicyEpochRef},{"namespace",s.NamespaceRef},
		{"subject",s.SubjectRef},{"claim",s.ClaimID},{"query profile",s.QueryProfileRef},
	}
	for _, f := range fields { if strings.TrimSpace(f.value)=="" { return "",fmt.Errorf("empty %s",f.name) } }
	var b bytes.Buffer
	writeString(&b,"MYCELIX-TEMPORAL-QUERY-SCOPE-V1")
	writeString(&b,s.VerifierRef); writeString(&b,s.PolicyEpochRef); writeString(&b,s.NamespaceRef)
	writeString(&b,s.SubjectRef); writeString(&b,s.ClaimID); writeString(&b,s.QueryProfileRef)
	_ = binary.Write(&b,binary.BigEndian,s.ValidAt.UnixSeconds)
	_ = binary.Write(&b,binary.BigEndian,s.ValidAt.Nanoseconds)
	d:=sha256.Sum256(b.Bytes())
	return hex.EncodeToString(d[:]),nil
}

// CandidateRoot commits to the supplied candidate/disposition inventory. It does not prove
// that the supplied inventory is complete; an independent source-universe verifier owns that.
func CandidateRoot(candidates []OwnerCandidate) (string,error) {
	ordered:=append([]OwnerCandidate(nil),candidates...)
	sort.Slice(ordered,func(i,j int)bool{return ordered[i].OwnerRef<ordered[j].OwnerRef})
	seenHistory:=make(map[string]string,len(ordered))
	var b bytes.Buffer
	writeString(&b,"MYCELIX-TEMPORAL-OWNER-UNIVERSE-V1")
	_ = binary.Write(&b,binary.BigEndian,uint32(len(ordered)))
	for i,c:=range ordered {
		if strings.TrimSpace(c.OwnerRef)=="" || (i>0 && ordered[i-1].OwnerRef==c.OwnerRef) { return "",errors.New("empty or duplicate owner candidate") }
		if err:=validateCandidate(c); err!=nil { return "",err }
		if c.Disposition==CandidateRequired {
			if prior,ok:=seenHistory[c.HistoryRef]; ok { return "",fmt.Errorf("owner-history identity %q is shared by %q and %q",c.HistoryRef,prior,c.OwnerRef) }
			seenHistory[c.HistoryRef]=c.OwnerRef
		}
		writeString(&b,c.OwnerRef); writeString(&b,c.HistoryRef); writeString(&b,string(c.Disposition))
		writeString(&b,c.ExclusionCode); writeString(&b,c.ExclusionProof)
	}
	d:=sha256.Sum256(b.Bytes())
	return hex.EncodeToString(d[:]),nil
}
func validateCandidate(c OwnerCandidate) error {
	switch c.Disposition {
	case CandidateRequired:
		if strings.TrimSpace(c.HistoryRef)=="" { return fmt.Errorf("required owner %q lacks stable history identity",c.OwnerRef) }
		if c.ExclusionCode!="" || c.ExclusionProof!="" { return fmt.Errorf("required owner %q carries exclusion fields",c.OwnerRef) }
	case CandidateExcluded:
		if strings.TrimSpace(c.ExclusionCode)=="" || !validDigest(c.ExclusionProof) { return fmt.Errorf("excluded owner %q lacks profile reason or evidence digest",c.OwnerRef) }
	case CandidateUnknown,CandidateWithheld,CandidateOffline:
		if c.ExclusionCode!="" || c.ExclusionProof!="" { return fmt.Errorf("unresolved owner %q carries exclusion fields",c.OwnerRef) }
	default: return fmt.Errorf("unknown candidate disposition for %q",c.OwnerRef)
	}
	return nil
}

// Reconcile compares a closure manifest to an independently supplied universe snapshot, then
// verifies that the frontier represents every Required owner with the same stable history ID.
// It intentionally returns only structural dispositions. Source-universe and checkpoint
// authentication remain NotEvaluated: this package verifies neither signatures nor completeness.
func Reconcile(scope QueryScope, manifest ScopeClosureManifest, universe UniverseSnapshot, frontier FrontierSnapshot) Result {
	scopeDigest,err:=ScopeDigest(scope)
	if err!=nil { return failed(scopeDigest,"invalid query scope: "+err.Error(),manifest,frontier) }
	root,err:=CandidateRoot(universe.Candidates)
	if err!=nil { return failed(scopeDigest,"invalid independently supplied universe inventory: "+err.Error(),manifest,frontier) }
	manifestRoot,err:=CandidateRoot(manifest.Candidates)
	if err!=nil { return failed(scopeDigest,"invalid closure manifest inventory: "+err.Error(),manifest,frontier) }
	base:=Result{
		Disposition:Indeterminate, ScopeDigest:scopeDigest, CandidateRoot:root, FrontierDigest:frontier.FrontierDigest,
		RegistryRef:manifest.RegistryRef, RegistryEpoch:manifest.RegistryEpoch, UniverseProfile:manifest.ProfileRef,
		SourceProofDigest:manifest.SourceProofDigest, UniverseAuthentication:"NotEvaluated", FrontierAuthentication:"NotEvaluated",
	}
	if manifest.ScopeDigest!=scopeDigest || universe.ScopeDigest!=scopeDigest || frontier.ScopeDigest!=scopeDigest {
		return finalize(base,"scope digest substitution or mismatch")
	}
	if manifest.RegistryRef=="" || manifest.RegistryEpoch=="" || manifest.ProfileRef=="" ||
		manifest.RegistryRef!=universe.RegistryRef || manifest.RegistryEpoch!=universe.RegistryEpoch || manifest.ProfileRef!=universe.ProfileRef {
		return finalize(base,"registry, epoch or owner-universe profile mismatch")
	}
	if manifest.CandidateRoot!=root || manifest.CandidateRoot!=manifestRoot || manifest.SourceProofDigest=="" ||
		manifest.SourceProofDigest!=universe.SourceProofDigest || !validDigest(universe.SourceProofDigest) {
		return finalize(base,"candidate inventory or source-proof commitment mismatch")
	}
	if frontier.VerifierRef!=scope.VerifierRef || frontier.PolicyEpochRef!=scope.PolicyEpochRef ||
		frontier.FrontierDigest=="" || !validDigest(frontier.FrontierDigest) {
		return finalize(base,"frontier verifier, policy or digest mismatch")
	}
	if !sameCandidates(universe.Candidates,manifest.Candidates) { return finalize(base,"manifest candidates differ from independently supplied universe inventory") }

	candidateByOwner:=make(map[string]OwnerCandidate,len(universe.Candidates))
	for _,c:=range universe.Candidates {
		candidateByOwner[c.OwnerRef]=c
		switch c.Disposition {
		case CandidateUnknown: return finalize(base,"owner universe contains Unknown owner: "+c.OwnerRef)
		case CandidateWithheld: return finalize(base,"owner universe contains Withheld owner: "+c.OwnerRef)
		case CandidateOffline: return finalize(base,"owner universe contains Unavailable owner: "+c.OwnerRef)
		case CandidateRequired: base.RequiredOwnerCount++
		case CandidateExcluded: base.ExcludedOwnerCount++
		}
	}
	frontierByOwner:=make(map[string]OwnerFrontierBinding,len(frontier.Owners))
	for _,o:=range frontier.Owners {
		if strings.TrimSpace(o.OwnerRef)=="" || o.HistoryRef=="" || !validDigest(o.LogRootDigest) { return finalize(base,"malformed frontier owner binding") }
		if _,ok:=frontierByOwner[o.OwnerRef]; ok { return finalize(base,"duplicate owner in frontier: "+o.OwnerRef) }
		if len(o.Gaps)!=0 { return finalize(base,"unresolved owner-history gaps: "+o.OwnerRef) }
		frontierByOwner[o.OwnerRef]=o
	}
	for ref,c:=range candidateByOwner {
		binding,exists:=frontierByOwner[ref]
		if c.Disposition==CandidateRequired {
			if !exists { return finalize(base,"required owner omitted from accepted frontier: "+ref) }
			if binding.HistoryRef=="" || binding.HistoryRef!=c.HistoryRef { return finalize(base,"required owner history identity differs from universe roster: "+ref) }
		} else if c.Disposition==CandidateExcluded && exists {
			return finalize(base,"profile-excluded owner unexpectedly appears in frontier: "+ref)
		}
	}
	for ref,c:=range frontierByOwner {
		candidate,exists:=candidateByOwner[ref]
		if !exists || candidate.Disposition!=CandidateRequired { return finalize(base,"frontier contains owner not required by declared universe: "+ref) }
	}
	base.Disposition=FrontierCoversSuppliedRoster
	return finalize(base,"frontier roster matches supplied expected universe; source-universe and checkpoint authentication remain external")
}
func sameCandidates(a,b []OwnerCandidate) bool {
	left:=append([]OwnerCandidate(nil),a...); right:=append([]OwnerCandidate(nil),b...)
	sort.Slice(left,func(i,j int)bool{return left[i].OwnerRef<left[j].OwnerRef})
	sort.Slice(right,func(i,j int)bool{return right[i].OwnerRef<right[j].OwnerRef})
	if len(left)!=len(right) { return false }
	for i:=range left { if left[i]!=right[i] { return false } }
	return true
}
func failed(scopeDigest,reason string,manifest ScopeClosureManifest,frontier FrontierSnapshot) Result {
	root,_:=CandidateRoot(manifest.Candidates)
	return finalize(Result{
		Disposition:Indeterminate, ScopeDigest:scopeDigest, CandidateRoot:root, FrontierDigest:frontier.FrontierDigest,
		RegistryRef:manifest.RegistryRef, RegistryEpoch:manifest.RegistryEpoch, UniverseProfile:manifest.ProfileRef,
		SourceProofDigest:manifest.SourceProofDigest, UniverseAuthentication:"NotEvaluated", FrontierAuthentication:"NotEvaluated",
	},reason)
}
func finalize(r Result,reason string) Result {
	r.Reason=reason
	var b bytes.Buffer
	writeString(&b,"MYCELIX-TEMPORAL-SCOPE-CLOSURE-RESULT-V1")
	writeString(&b,string(r.Disposition)); writeString(&b,r.Reason); writeString(&b,r.ScopeDigest)
	writeString(&b,r.CandidateRoot); writeString(&b,r.FrontierDigest); writeString(&b,r.RegistryRef)
	writeString(&b,r.RegistryEpoch); writeString(&b,r.UniverseProfile); writeString(&b,r.SourceProofDigest)
	_ = binary.Write(&b,binary.BigEndian,uint64(r.RequiredOwnerCount))
	_ = binary.Write(&b,binary.BigEndian,uint64(r.ExcludedOwnerCount))
	writeString(&b,r.UniverseAuthentication); writeString(&b,r.FrontierAuthentication)
	d:=sha256.Sum256(b.Bytes()); r.CertificateDigest=hex.EncodeToString(d[:])
	return r
}
func validDigest(s string) bool {
	if len(s)!=64 || s!=strings.ToLower(s) { return false }
	_,err:=hex.DecodeString(s); return err==nil
}
func writeString(b *bytes.Buffer,s string) { _=binary.Write(b,binary.BigEndian,uint32(len(s))); _,_=b.WriteString(s) }
