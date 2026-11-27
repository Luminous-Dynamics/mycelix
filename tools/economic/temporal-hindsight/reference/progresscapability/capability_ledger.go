package progresscapability

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

const (
	MaxIdentifierBytes = 4096
	MaxOwners = 100000
	MaxLedgerEvents = 1000000
	MaxActiveCapabilities = 100000
	MaxLedgerOwnerCoordinates = 5000000
	MaxLedgerEncodingBytes uint64 = 67108864
	OutcomeCommitted CapabilityOutcomeV1 = "Committed"
	OutcomeUnknown CapabilityOutcomeV1 = "OutcomeUnknown"
	OutcomeApplied CapabilityOutcomeV1 = "ReconciledApplied"
	OutcomeNotApplied CapabilityOutcomeV1 = "ReconciledNotApplied"
	TransitionMint CapabilityTransitionKindV1 = "Mint"
	TransitionAdvance CapabilityTransitionKindV1 = "Advance"
	TransitionRelease CapabilityTransitionKindV1 = "Release"
	TransitionRevoke CapabilityTransitionKindV1 = "Revoke"
	LedgerValid LedgerDispositionV1 = "LedgerStructurallyValid"
	LedgerIndeterminate LedgerDispositionV1 = "Indeterminate"
	ProgressDerived ProgressDispositionV1 = "ProgressAntichainDerived"
	ProgressDeclaredClosed ProgressDispositionV1 = "NoOutstandingDeclaredCapabilities"
	ProgressUnavailable ProgressDispositionV1 = "Indeterminate"
	NotEvaluated = "NotEvaluated"
)

type CapabilityOutcomeV1 string
type CapabilityTransitionKindV1 string
type LedgerDispositionV1 string
type ProgressDispositionV1 string

type CapabilityScopeV1 struct {
	SourceRef string
	RegistryRef string
	RegistryEpoch string
	ProfileRef string
	QueryScopeDigest string
	OwnerRefs []string
}

type OwnerCoordinateV1 struct {
	OwnerRef string
	Sequence uint64
}

type CapabilityTransitionV1 struct {
	Kind CapabilityTransitionKindV1
	CapabilityID string
	Generation uint64
	PriorCapabilityDigest string
	OwnerCut []OwnerCoordinateV1
	IssuerRef string
	IssuerKeyEpoch string
	AuthorityGrantRef string
	CheckpointRef string
}

type CapabilityEventV1 struct {
	LogSequence uint64
	EventID string
	PreviousEventDigest string
	Outcome CapabilityOutcomeV1
	Transition *CapabilityTransitionV1
	ReconcilesEventID string
	EventDigest string
}

type ActiveCapabilityV1 struct {
	CapabilityID string
	Generation uint64
	TransitionDigest string
	OwnerCut []OwnerCoordinateV1
	IssuerRef string
	IssuerKeyEpoch string
	AuthorityGrantRef string
	CheckpointRef string
}

type ProgressPointV1 struct {
	OwnerCut []OwnerCoordinateV1
	PointDigest string
}

type ReplayResultV1 struct {
	Disposition LedgerDispositionV1
	Reason string
	ScopeDigest string
	ExpectedEventCount uint64
	ObservedEventCount uint64
	GenesisDigest string
	ObservedTailDigest string
	ActiveCapabilities []ActiveCapabilityV1
	PendingEventID string
	SubmittedEventsDigest string
	MutationBlocked bool
	LedgerAuthentication string
	IssuerAuthorityAuthentication string
	CheckpointAuthentication string
	SourceUniverseCompleteness string
	CertificateDigest string
}

type ProgressResultV1 struct {
	Disposition ProgressDispositionV1
	Reason string
	ScopeDigest string
	LedgerCertificateDigest string
	ProgressPoints []ProgressPointV1
	ProgressAntichainDigest string
	ProgressAuthentication string
	SourceUniverseCompleteness string
	CertificateDigest string
}

type activeState struct {
	transition CapabilityTransitionV1
	digest string
	cut []uint64
}

func writeUint32(b *bytes.Buffer, value uint32) {
	var encoded [4]byte
	binary.BigEndian.PutUint32(encoded[:], value)
	_, _ = b.Write(encoded[:])
}

func writeUint64(b *bytes.Buffer, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	_, _ = b.Write(encoded[:])
}

func writeString(b *bytes.Buffer, value string) {
	writeUint32(b, uint32(len(value)))
	_, _ = b.WriteString(value)
}

func digestBytes(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}

func validDigest(value string) bool {
	if len(value) != 64 || strings.ToLower(value) != value { return false }
	_, err := hex.DecodeString(value)
	return err == nil
}

func canonicalOwners(scope CapabilityScopeV1) ([]string, error) {
	fields := []string{scope.SourceRef, scope.RegistryRef, scope.RegistryEpoch, scope.ProfileRef}
	for _, field := range fields {
		if strings.TrimSpace(field) == "" { return nil, errors.New("capability scope identity is incomplete") }
		if len(field) > MaxIdentifierBytes { return nil, errors.New("capability scope identifier exceeds byte limit") }
	}
	if len(scope.OwnerRefs) > MaxOwners { return nil, errors.New("capability owner universe exceeds limit") }
	if !validDigest(scope.QueryScopeDigest) { return nil, errors.New("query scope digest malformed") }
	owners := append([]string(nil), scope.OwnerRefs...)
	sort.Strings(owners)
	for i, owner := range owners {
		if strings.TrimSpace(owner) == "" || len(owner) > MaxIdentifierBytes { return nil, errors.New("invalid owner reference") }
		if i > 0 && owners[i-1] == owner { return nil, errors.New("duplicate owner reference") }
	}
	return owners, nil
}

func ScopeDigestV1(scope CapabilityScopeV1) (string, error) {
	owners, err := canonicalOwners(scope)
	if err != nil { return "", err }
	var b bytes.Buffer
	writeString(&b, "MYCELIX-CAPABILITY-SCOPE-V1")
	for _, value := range []string{scope.SourceRef,scope.RegistryRef,scope.RegistryEpoch,scope.ProfileRef,scope.QueryScopeDigest} { writeString(&b,value) }
	_ = writeUint32(&b,uint32(len(owners)))
	for _, owner := range owners { writeString(&b,owner) }
	return digestBytes(b.Bytes()),nil
}

func genesisDigest(scopeDigest string) string {
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-LEDGER-GENESIS-V1")
	writeString(&b,scopeDigest)
	return digestBytes(b.Bytes())
}

func cutVector(cut []OwnerCoordinateV1, owners []string) ([]uint64,error) {
	if len(cut)!=len(owners) { return nil,errors.New("owner cut does not exactly cover declared owner universe") }
	valuesByOwner:=make(map[string]uint64,len(cut))
	for _,coordinate:=range cut {
		if strings.TrimSpace(coordinate.OwnerRef)=="" || len(coordinate.OwnerRef)>MaxIdentifierBytes { return nil,errors.New("invalid owner cut coordinate") }
		if _,exists:=valuesByOwner[coordinate.OwnerRef];exists{return nil,errors.New("duplicate owner cut coordinate")}
		valuesByOwner[coordinate.OwnerRef]=coordinate.Sequence
	}
	values:=make([]uint64,len(owners))
	for i,owner:=range owners {
		value,ok:=valuesByOwner[owner];if !ok{return nil,errors.New("owner cut omits required owner")}
		values[i]=value
	}
	if len(valuesByOwner)!=len(owners){return nil,errors.New("owner cut contains foreign owner")}
	return values,nil
}

func cutFromVector(owners []string, vector []uint64) []OwnerCoordinateV1 {
	result:=make([]OwnerCoordinateV1,len(owners))
	for i,owner:=range owners { result[i]=OwnerCoordinateV1{OwnerRef:owner,Sequence:vector[i]} }
	return result
}

func transitionDigest(scopeDigest string, transition CapabilityTransitionV1) string {
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-TRANSITION-V1")
	writeString(&b,scopeDigest)
	writeString(&b,string(transition.Kind))
	writeString(&b,transition.CapabilityID)
	_ = writeUint64(&b,transition.Generation)
	writeString(&b,transition.PriorCapabilityDigest)
	coords:=append([]OwnerCoordinateV1(nil),transition.OwnerCut...)
	sort.Slice(coords,func(i,j int)bool{
		if coords[i].OwnerRef==coords[j].OwnerRef{return coords[i].Sequence<coords[j].Sequence}
		return coords[i].OwnerRef<coords[j].OwnerRef
	})
	_ = writeUint32(&b,uint32(len(coords)))
	for _,coordinate:=range coords { writeString(&b,coordinate.OwnerRef);_ = writeUint64(&b,coordinate.Sequence) }
	for _,value:=range []string{transition.IssuerRef,transition.IssuerKeyEpoch,transition.AuthorityGrantRef,transition.CheckpointRef}{writeString(&b,value)}
	return digestBytes(b.Bytes())
}

func EventDigestV1(scopeDigest string,event CapabilityEventV1)(string,error){
	if event.LogSequence==0 || strings.TrimSpace(event.EventID)=="" || len(event.EventID)>MaxIdentifierBytes{return "",errors.New("invalid event sequence or ID")}
	if !validDigest(scopeDigest) || !validDigest(event.PreviousEventDigest){return "",errors.New("invalid scope or previous event digest")}
	if len(event.ReconcilesEventID)>MaxIdentifierBytes{return "",errors.New("reconciliation event reference exceeds byte limit")}
	if event.Transition!=nil {
		transition:=event.Transition
		for _,field:=range []string{transition.CapabilityID,transition.IssuerRef,transition.IssuerKeyEpoch,transition.AuthorityGrantRef,transition.CheckpointRef,transition.PriorCapabilityDigest} {
			if len(field)>MaxIdentifierBytes{return "",errors.New("transition field exceeds byte limit")}
		}
		if len(transition.OwnerCut)>MaxOwners{return "",errors.New("transition owner cut exceeds owner limit")}
	}
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-EVENT-V1")
	writeString(&b,scopeDigest)
	_ = writeUint64(&b,event.LogSequence)
	writeString(&b,event.EventID)
	writeString(&b,event.PreviousEventDigest)
	writeString(&b,string(event.Outcome))
	if event.Transition==nil { b.WriteByte(0) } else { b.WriteByte(1);writeString(&b,transitionDigest(scopeDigest,*event.Transition)) }
	writeString(&b,event.ReconcilesEventID)
	return digestBytes(b.Bytes()),nil
}

func cloneTransition(value CapabilityTransitionV1) CapabilityTransitionV1 {
	value.OwnerCut=append([]OwnerCoordinateV1(nil),value.OwnerCut...)
	return value
}

func validateTransition(scopeDigest string,owners []string,transition CapabilityTransitionV1,active map[string]activeState,usedIDs map[string]struct{})([]uint64,string,error){
	fields:=[]string{transition.CapabilityID,transition.IssuerRef,transition.IssuerKeyEpoch,transition.AuthorityGrantRef,transition.CheckpointRef}
	for _,field:=range fields {if strings.TrimSpace(field)=="" || len(field)>MaxIdentifierBytes{return nil,"",errors.New("capability transition identity or authority reference invalid")}}
	cut,err:=cutVector(transition.OwnerCut,owners);if err!=nil{return nil,"",err}
	current,exists:=active[transition.CapabilityID]
	switch transition.Kind {
	case TransitionMint:
		if _,everUsed:=usedIDs[transition.CapabilityID];everUsed{return nil,"",errors.New("capability ID cannot be reused")}
		if len(active)>=MaxActiveCapabilities{return nil,"",errors.New("active capability limit exceeded")}
		if transition.Generation!=1 || transition.PriorCapabilityDigest!=""{return nil,"",errors.New("mint must start at generation one without predecessor")}
	case TransitionAdvance:
		if !exists{return nil,"",errors.New("cannot advance an inactive capability")}
		if transition.Generation!=current.transition.Generation+1 || transition.PriorCapabilityDigest!=current.digest{return nil,"",errors.New("capability advance generation or predecessor mismatch")}
		for i:=range cut {if cut[i]<current.cut[i]{return nil,"",errors.New("capability lower-bound cut regressed")}}
	case TransitionRelease,TransitionRevoke:
		if !exists{return nil,"",errors.New("release/revoke is not consume-once: capability is inactive")}
		if transition.Generation!=current.transition.Generation+1 || transition.PriorCapabilityDigest!=current.digest{return nil,"",errors.New("release/revoke generation or predecessor mismatch")}
		for i:=range cut {if cut[i]!=current.cut[i]{return nil,"",errors.New("release/revoke cannot rewrite the capability lower-bound cut")}}
	default:
		return nil,"",errors.New("unknown capability transition kind")
	}
	return cut,transitionDigest(scopeDigest,transition),nil
}

func rawEventsDigest(events []CapabilityEventV1) string {
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-RAW-EVENTS-V1")
	_ = writeUint32(&b,uint32(len(events)))
	for _,event:=range events {
		_ = writeUint64(&b,event.LogSequence)
		writeString(&b,event.EventID);writeString(&b,event.PreviousEventDigest);writeString(&b,string(event.Outcome))
		writeString(&b,event.ReconcilesEventID);writeString(&b,event.EventDigest)
		if event.Transition!=nil {writeString(&b,transitionDigest("raw",*event.Transition))}
	}
	return digestBytes(b.Bytes())
}

func finalizeReplay(result ReplayResultV1) ReplayResultV1 {
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-REPLAY-RESULT-V1")
	writeString(&b,string(result.Disposition));writeString(&b,result.Reason);writeString(&b,result.ScopeDigest)
	writeUint64(&b,result.ExpectedEventCount);writeUint64(&b,result.ObservedEventCount)
	writeString(&b,result.GenesisDigest);writeString(&b,result.ObservedTailDigest)
	writeString(&b,result.PendingEventID);writeString(&b,result.SubmittedEventsDigest)
	_ = writeUint32(&b,uint32(len(result.ActiveCapabilities)))
	for _,cap:=range result.ActiveCapabilities {writeString(&b,cap.CapabilityID);_ = writeUint64(&b,cap.Generation);writeString(&b,cap.TransitionDigest)}
	writeString(&b,result.LedgerAuthentication);writeString(&b,result.IssuerAuthorityAuthentication)
	writeString(&b,result.CheckpointAuthentication);writeString(&b,result.SourceUniverseCompleteness)
	result.CertificateDigest=digestBytes(b.Bytes())
	return result
}

func failedReplay(result ReplayResultV1,reason string)ReplayResultV1 {
	result.Disposition=LedgerIndeterminate;result.Reason=reason
	return finalizeReplay(result)
}

// ReplayCapabilityLedgerV1 reconstructs active capability heads from an
// append-only chain and supplied event-count/tail anchor. It validates only
// structure; signatures, grants, checkpoint roots and source completeness
// remain NotEvaluated.
func ReplayCapabilityLedgerV1(scope CapabilityScopeV1,events []CapabilityEventV1,expectedCount uint64,expectedTail string)ReplayResultV1{
	result:=ReplayResultV1{Disposition:LedgerIndeterminate,ExpectedEventCount:expectedCount,ObservedEventCount:uint64(len(events)),
		LedgerAuthentication:NotEvaluated,IssuerAuthorityAuthentication:NotEvaluated,CheckpointAuthentication:NotEvaluated,SourceUniverseCompleteness:NotEvaluated}
	if expectedCount>MaxLedgerEvents || uint64(len(events))>MaxLedgerEvents {
		result.SubmittedEventsDigest=digestBytes([]byte(fmt.Sprintf("MYCELIX-OVERLIMIT-EVENT-SUBMISSION-V1|expected=%d|observed=%d",expectedCount,len(events))))
		return failedReplay(result,"ledger event count exceeds configured limit")
	}
	var totalCoordinates uint64
	var totalEncodingBytes uint64
	addEncodedField := func(value string) bool {
		if len(value)>MaxIdentifierBytes || uint64(len(value))>MaxLedgerEncodingBytes-totalEncodingBytes { return false }
		totalEncodingBytes+=uint64(len(value))
		return true
	}
	for _,event:=range events {
		for _,field:=range []string{event.EventID,event.PreviousEventDigest,string(event.Outcome),event.ReconcilesEventID,event.EventDigest} {
			if !addEncodedField(field) { return failedReplay(result,"ledger event encoding byte budget exceeded") }
		}
		if event.Transition==nil {continue}
		if len(event.Transition.OwnerCut)>MaxOwners { return failedReplay(result,"transition owner cut exceeds owner limit") }
		count:=uint64(len(event.Transition.OwnerCut))
		if count>MaxLedgerOwnerCoordinates-totalCoordinates{return failedReplay(result,"ledger owner-coordinate work budget exceeded")}
		totalCoordinates+=count
		for _,field:=range []string{string(event.Transition.Kind),event.Transition.CapabilityID,event.Transition.PriorCapabilityDigest,
			event.Transition.IssuerRef,event.Transition.IssuerKeyEpoch,event.Transition.AuthorityGrantRef,event.Transition.CheckpointRef} {
			if !addEncodedField(field) { return failedReplay(result,"ledger transition encoding byte budget exceeded") }
		}
		for _,coordinate:=range event.Transition.OwnerCut {
			if !addEncodedField(coordinate.OwnerRef) { return failedReplay(result,"ledger owner-coordinate encoding byte budget exceeded") }
		}
	}
	result.SubmittedEventsDigest=rawEventsDigest(events)
	if !validDigest(expectedTail){return failedReplay(result,"expected tail anchor is malformed")}
	if uint64(len(events))!=expectedCount{return failedReplay(result,"observed event count differs from independent tail anchor")}
	scopeDigest,err:=ScopeDigestV1(scope);if err!=nil{return failedReplay(result,"invalid scope: "+err.Error())}
	owners,_:=canonicalOwners(scope);result.ScopeDigest=scopeDigest
	genesis:=genesisDigest(scopeDigest);result.GenesisDigest=genesis;result.ObservedTailDigest=genesis
	active:=make(map[string]activeState)
	usedCapabilityIDs:=make(map[string]struct{})
	seenIDs:=make(map[string]struct{},len(events))
	running:=genesis
	var pending *CapabilityEventV1
	for index,event:=range events{
		expectedSequence:=uint64(index)+1
		if event.LogSequence!=expectedSequence{return failedReplay(result,fmt.Sprintf("event sequence discontinuity at index %d",index))}
		if strings.TrimSpace(event.EventID)=="" || len(event.EventID)>MaxIdentifierBytes{return failedReplay(result,"invalid event ID")}
		if _,exists:=seenIDs[event.EventID];exists{return failedReplay(result,"duplicate event ID")}
		seenIDs[event.EventID]=struct{}{}
		if event.PreviousEventDigest!=running{return failedReplay(result,fmt.Sprintf("previous event digest mismatch at sequence %d",expectedSequence))}
		calculated,err:=EventDigestV1(scopeDigest,event);if err!=nil{return failedReplay(result,err.Error())}
		if calculated!=event.EventDigest{return failedReplay(result,fmt.Sprintf("event digest mismatch at sequence %d",expectedSequence))}
		if pending!=nil && event.Outcome!=OutcomeApplied && event.Outcome!=OutcomeNotApplied{return failedReplay(result,"mutation attempted while outcome is unknown; reconcile first")}
		switch event.Outcome{
		case OutcomeCommitted:
			if pending!=nil || event.Transition==nil || event.ReconcilesEventID!=""{return failedReplay(result,"malformed committed transition event")}
			cut,digest,err:=validateTransition(scopeDigest,owners,*event.Transition,active,usedCapabilityIDs);if err!=nil{return failedReplay(result,err.Error())}
			applyTransition(active,usedCapabilityIDs,*event.Transition,cut,digest)
		case OutcomeUnknown:
			if pending!=nil || event.Transition==nil || event.ReconcilesEventID!=""{return failedReplay(result,"malformed or nested unknown-outcome event")}
			if _,_,err:=validateTransition(scopeDigest,owners,*event.Transition,active,usedCapabilityIDs);err!=nil{return failedReplay(result,"unknown transition intent is invalid: "+err.Error())}
			copyEvent:=event;pending=&copyEvent
		case OutcomeApplied,OutcomeNotApplied:
			if event.Transition!=nil || pending==nil || event.ReconcilesEventID!=pending.EventID{return failedReplay(result,"reconciliation does not match the pending unknown outcome")}
			if event.Outcome==OutcomeApplied{
				cut,digest,err:=validateTransition(scopeDigest,owners,*pending.Transition,active,usedCapabilityIDs);if err!=nil{return failedReplay(result,"reconciled transition no longer valid: "+err.Error())}
				applyTransition(active,usedCapabilityIDs,*pending.Transition,cut,digest)
			}
			pending=nil
		default:
			return failedReplay(result,"unknown capability event outcome")
		}
		running=calculated
	}
	result.ObservedTailDigest=running
	if running!=expectedTail{return failedReplay(result,"observed event tail differs from independent anchor")}
	if pending!=nil{
		result.PendingEventID=pending.EventID;result.MutationBlocked=true
		result.Reason="capability outcome is unknown; no further mutation or progress derivation until reconciliation"
	}else{result.Disposition=LedgerValid;result.Reason="capability event stream is structurally valid and replayed to its anchored tail"}
	for id,state:=range active{
		result.ActiveCapabilities=append(result.ActiveCapabilities,ActiveCapabilityV1{CapabilityID:id,Generation:state.transition.Generation,TransitionDigest:state.digest,
			OwnerCut:cutFromVector(owners,state.cut),IssuerRef:state.transition.IssuerRef,IssuerKeyEpoch:state.transition.IssuerKeyEpoch,
			AuthorityGrantRef:state.transition.AuthorityGrantRef,CheckpointRef:state.transition.CheckpointRef})
	}
	sort.Slice(result.ActiveCapabilities,func(i,j int)bool{return result.ActiveCapabilities[i].CapabilityID<result.ActiveCapabilities[j].CapabilityID})
	return finalizeReplay(result)
}

func applyTransition(active map[string]activeState,usedIDs map[string]struct{},transition CapabilityTransitionV1,cut []uint64,digest string){
	switch transition.Kind{
	case TransitionMint:
		usedIDs[transition.CapabilityID]=struct{}{}
		active[transition.CapabilityID]=activeState{transition:cloneTransition(transition),digest:digest,cut:append([]uint64(nil),cut...)}
	case TransitionAdvance:
		active[transition.CapabilityID]=activeState{transition:cloneTransition(transition),digest:digest,cut:append([]uint64(nil),cut...)}
	case TransitionRelease,TransitionRevoke:
		delete(active,transition.CapabilityID)
	}
}

func coordinatesLessOrEqual(a,b []uint64)bool{
	if len(a)!=len(b){return false}
	for i:=range a{if a[i]>b[i]{return false}}
	return true
}

func pointDigest(scopeDigest string,owners []string,cut []uint64)string{
	var b bytes.Buffer
	writeString(&b,"MYCELIX-DERIVED-PROGRESS-POINT-V1");writeString(&b,scopeDigest)
	for i,owner:=range owners{writeString(&b,owner);_ = writeUint64(&b,cut[i])}
	return digestBytes(b.Bytes())
}

func deriveProgressDigest(scopeDigest string,points []ProgressPointV1)string{
	var b bytes.Buffer
	writeString(&b,"MYCELIX-DERIVED-PROGRESS-ANTICHAIN-V1");writeString(&b,scopeDigest)
	_ = writeUint32(&b,uint32(len(points)))
	for _,point:=range points{writeString(&b,point.PointDigest)}
	return digestBytes(b.Bytes())
}

func finalizeProgress(result ProgressResultV1)ProgressResultV1{
	var b bytes.Buffer
	writeString(&b,"MYCELIX-CAPABILITY-PROGRESS-RESULT-V1")
	writeString(&b,string(result.Disposition));writeString(&b,result.Reason);writeString(&b,result.ScopeDigest)
	writeString(&b,result.LedgerCertificateDigest);writeString(&b,result.ProgressAntichainDigest)
	for _,point:=range result.ProgressPoints{writeString(&b,point.PointDigest)}
	writeString(&b,result.ProgressAuthentication);writeString(&b,result.SourceUniverseCompleteness)
	result.CertificateDigest=digestBytes(b.Bytes());return result
}

// DeriveProgressAntichainV1 derives minimal outstanding-capability cuts. A
// pending unknown result blocks derivation; empty means only no declared
// capability remains, never that source completion is authenticated.
func DeriveProgressAntichainV1(scope CapabilityScopeV1,replay ReplayResultV1)ProgressResultV1{
	result:=ProgressResultV1{Disposition:ProgressUnavailable,Reason:"progress derivation is indeterminate",
		ProgressAuthentication:NotEvaluated,SourceUniverseCompleteness:NotEvaluated,
		ScopeDigest:replay.ScopeDigest,LedgerCertificateDigest:replay.CertificateDigest}
	if replay.Disposition!=LedgerValid || replay.MutationBlocked{
		result.Reason="ledger is not fully reconciled and structurally anchored; progress derivation is blocked"
		return finalizeProgress(result)
	}
	scopeDigest,err:=ScopeDigestV1(scope);if err!=nil{result.Reason=err.Error();return finalizeProgress(result)}
	if replay.ScopeDigest!=scopeDigest{result.Reason="replay scope does not match progress derivation scope";return finalizeProgress(result)}
	owners,_:=canonicalOwners(scope)
	vectors:=make([][]uint64,0,len(replay.ActiveCapabilities))
	for _,cap:=range replay.ActiveCapabilities{
		vector,err:=cutVector(cap.OwnerCut,owners);if err!=nil{result.Reason="active capability has malformed cut: "+err.Error();return finalizeProgress(result)}
		duplicateOrDominated:=false
		for i:=0;i<len(vectors);{
			if coordinatesLessOrEqual(vectors[i],vector){duplicateOrDominated=true;break}
			if coordinatesLessOrEqual(vector,vectors[i]){vectors=append(vectors[:i],vectors[i+1:]...);continue}
			i++
		}
		if !duplicateOrDominated{vectors=append(vectors,vector)}
	}
	for _,vector:=range vectors{result.ProgressPoints=append(result.ProgressPoints,ProgressPointV1{OwnerCut:cutFromVector(owners,vector),PointDigest:pointDigest(scopeDigest,owners,vector)})}
	sort.Slice(result.ProgressPoints,func(i,j int)bool{return result.ProgressPoints[i].PointDigest<result.ProgressPoints[j].PointDigest})
	result.ProgressAntichainDigest=deriveProgressDigest(scopeDigest,result.ProgressPoints)
	if len(result.ProgressPoints)==0{
		result.Disposition=ProgressDeclaredClosed
		result.Reason="no outstanding declared capability remains; source authentication and owner-universe completeness are not evaluated"
	}else{
		result.Disposition=ProgressDerived
		result.Reason="progress antichain derived from structurally replayed active capability heads"
	}
	return finalizeProgress(result)
}
