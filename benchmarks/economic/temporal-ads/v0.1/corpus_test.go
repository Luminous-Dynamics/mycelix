package corpus

import (
	"bufio"
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"sort"
	"strings"
	"testing"

	hindsight "mycelix-temporal-hindsight-oracle"
	scopeclosure "mycelix-temporal-scope-closure"
)

type Instant struct {
	Seconds int64  `json:"seconds"`
	Nanos   uint32 `json:"nanos"`
}
type Scope struct {
	VerifierRef    string  `json:"verifier_ref"`
	PolicyEpochRef string  `json:"policy_epoch_ref"`
	NamespaceRef   string  `json:"namespace_ref"`
	SubjectRef     string  `json:"subject_ref"`
	ClaimID        string  `json:"claim_id"`
	QueryProfileRef string `json:"query_profile_ref"`
	ValidAt        Instant `json:"valid_at"`
}
type Candidate struct {
	OwnerRef             string `json:"owner_ref"`
	HistoryRef           string `json:"history_ref"`
	Disposition          string `json:"disposition"`
	ExclusionCode        string `json:"exclusion_code"`
	ExclusionProofDigest string `json:"exclusion_proof_digest"`
}
type Universe struct {
	RegistryRef       string      `json:"registry_ref"`
	RegistryEpoch     string      `json:"registry_epoch"`
	ProfileRef        string      `json:"profile_ref"`
	SourceProofDigest string      `json:"source_proof_digest"`
	Candidates        []Candidate `json:"candidates"`
}
type OwnerCut struct {
	OwnerRef     string `json:"owner_ref"`
	HistoryRef   string `json:"history_ref"`
	SequenceCut  uint64 `json:"sequence_cut"`
	LogRootDigest string `json:"log_root_digest"`
}
type Frontier struct {
	FrontierRef        string     `json:"frontier_ref"`
	FrontierDigest     string     `json:"frontier_digest"`
	RegistryRef        string     `json:"registry_ref"`
	RegistryEpoch      string     `json:"registry_epoch"`
	UniverseProfileRef string     `json:"universe_profile_ref"`
	Owners             []OwnerCut `json:"owners"`
}
type Event struct {
	Sequence     uint64  `json:"sequence"`
	EventID       string  `json:"event_id"`
	Dimension     string  `json:"dimension"`
	IntervalRef   string  `json:"interval_ref"`
	Value         *int64  `json:"value"`
	ExplicitOpen  bool    `json:"explicit_open"`
	Supersedes    string  `json:"supersedes"`
}
type OwnerLog struct {
	OwnerRef   string  `json:"owner_ref"`
	HistoryRef string  `json:"history_ref"`
	Events     []Event `json:"events"`
}
type ExpectedFrontier struct {
	Disposition           string   `json:"disposition"`
	MatchingIntervalRefs  []string `json:"matching_interval_refs"`
	ReasonCode            string   `json:"reason_code"`
}
type Expected struct {
	FrontierResults map[string]ExpectedFrontier `json:"frontier_results"`
	RosterDisposition string `json:"roster_disposition"`
	RosterReasonCode  string `json:"roster_reason_code"`
	HistoricalReplayEqual bool `json:"historical_replay_equal_across_suffix_variants"`
}
type Scenario struct {
	SchemaVersion string             `json:"schema_version"`
	ScenarioID    string             `json:"scenario_id"`
	Purpose       string             `json:"purpose"`
	Scope         Scope              `json:"scope"`
	Universe      Universe           `json:"universe"`
	Frontiers     map[string]Frontier `json:"frontiers"`
	OwnerLogs     []OwnerLog         `json:"owner_logs"`
	Expected      Expected           `json:"expected"`
}
type CorpusManifest struct {
	CorpusVersion string   `json:"corpus_version"`
	SchemaFile    string   `json:"schema_file"`
	ScenariosFile string   `json:"scenarios_file"`
	ScenarioCount int      `json:"scenario_count"`
	ScenarioIDs   []string `json:"scenario_ids"`
	Status        string   `json:"status"`
}

func loadCorpus(t *testing.T) ([]Scenario, CorpusManifest) {
	t.Helper()
	schemaBytes, err := os.ReadFile("scenario.schema.json")
	if err != nil { t.Fatal(err) }
	var schema map[string]any
	if err := json.Unmarshal(schemaBytes, &schema); err != nil { t.Fatalf("invalid JSON Schema JSON: %v", err) }
	if schema["$schema"] != "https://json-schema.org/draft/2020-12/schema" { t.Fatal("unexpected JSON Schema draft") }

	manifestBytes, err := os.ReadFile("manifest.json")
	if err != nil { t.Fatal(err) }
	var manifest CorpusManifest
	if err := json.Unmarshal(manifestBytes, &manifest); err != nil { t.Fatal(err) }

	file, err := os.Open("scenarios.jsonl")
	if err != nil { t.Fatal(err) }
	defer file.Close()
	scanner := bufio.NewScanner(file)
	scanner.Buffer(make([]byte, 4096), 4*1024*1024)
	var scenarios []Scenario
	for lineNo := 1; scanner.Scan(); lineNo++ {
		line := bytes.TrimSpace(scanner.Bytes())
		if len(line) == 0 { continue }
		decoder := json.NewDecoder(bytes.NewReader(line))
		decoder.DisallowUnknownFields()
		var scenario Scenario
		if err := decoder.Decode(&scenario); err != nil { t.Fatalf("scenario line %d: %v", lineNo, err) }
		var extra any
		if err := decoder.Decode(&extra); err != io.EOF { t.Fatalf("scenario line %d contains trailing JSON", lineNo) }
		scenarios = append(scenarios, scenario)
	}
	if err := scanner.Err(); err != nil { t.Fatal(err) }
	return scenarios, manifest
}

func validateScenario(s Scenario) error {
	if s.SchemaVersion != "MYCELIX-TEMPORAL-ADS-SCENARIO-V0.1" || s.ScenarioID == "" || s.Purpose == "" {
		return fmt.Errorf("missing or unsupported scenario identity")
	}
	if s.Scope.VerifierRef == "" || s.Scope.PolicyEpochRef == "" || s.Scope.NamespaceRef == "" ||
		s.Scope.SubjectRef == "" || s.Scope.ClaimID == "" || s.Scope.QueryProfileRef == "" ||
		s.Scope.ValidAt.Nanos >= 1_000_000_000 {
		return fmt.Errorf("invalid exact query scope")
	}
	if s.Universe.RegistryRef == "" || s.Universe.RegistryEpoch == "" || s.Universe.ProfileRef == "" || !digest(s.Universe.SourceProofDigest) {
		return fmt.Errorf("invalid owner-universe snapshot commitment")
	}
	candidates := make(map[string]Candidate, len(s.Universe.Candidates))
	required := make(map[string]bool)
	historyOwners := make(map[string]string)
	for _, candidate := range s.Universe.Candidates {
		if candidate.OwnerRef == "" { return fmt.Errorf("empty owner candidate") }
		if _, exists := candidates[candidate.OwnerRef]; exists { return fmt.Errorf("duplicate owner candidate %q", candidate.OwnerRef) }
		switch candidate.Disposition {
		case "Required":
			if candidate.HistoryRef == "" { return fmt.Errorf("required owner %q lacks history identity", candidate.OwnerRef) }
			if prior, exists := historyOwners[candidate.HistoryRef]; exists { return fmt.Errorf("shared history identity %q: %s / %s", candidate.HistoryRef, prior, candidate.OwnerRef) }
			historyOwners[candidate.HistoryRef] = candidate.OwnerRef
			required[candidate.OwnerRef] = true
		case "ExcludedByDeclaredProfile":
			if candidate.ExclusionCode == "" || !digest(candidate.ExclusionProofDigest) { return fmt.Errorf("unproven profile exclusion: %s", candidate.OwnerRef) }
		case "Unknown", "Withheld", "Unavailable":
		default:
			return fmt.Errorf("unsupported candidate disposition %q", candidate.Disposition)
		}
		candidates[candidate.OwnerRef] = candidate
	}
	logs := make(map[string]OwnerLog, len(s.OwnerLogs))
	globalEventIDs := make(map[string]bool)
	for _, log := range s.OwnerLogs {
		candidate, exists := candidates[log.OwnerRef]
		if !exists || log.OwnerRef == "" || log.HistoryRef == "" || candidate.HistoryRef != log.HistoryRef {
			return fmt.Errorf("owner log is not bound to a candidate identity: %s", log.OwnerRef)
		}
		if _, exists := logs[log.OwnerRef]; exists { return fmt.Errorf("duplicate owner log: %s", log.OwnerRef) }
		seen := make(map[string]Event, len(log.Events))
		for i, event := range log.Events {
			if event.Sequence != uint64(i+1) || event.EventID == "" || globalEventIDs[event.EventID] {
				return fmt.Errorf("non-contiguous sequence or duplicate event ID in %s", log.OwnerRef)
			}
			globalEventIDs[event.EventID] = true
			switch event.Dimension {
			case "VALID_FROM":
				if event.IntervalRef == "" || event.Value == nil || event.ExplicitOpen { return fmt.Errorf("malformed VALID_FROM: %s", event.EventID) }
			case "VALID_TO":
				if event.IntervalRef == "" || (event.ExplicitOpen && event.Value != nil) || (!event.ExplicitOpen && event.Value == nil) {
					return fmt.Errorf("malformed VALID_TO: %s", event.EventID)
				}
			case "NON_TEMPORAL":
				if event.IntervalRef != "" || event.Value != nil || event.ExplicitOpen || event.Supersedes != "" {
					return fmt.Errorf("non-temporal event carries interval fields: %s", event.EventID)
				}
			default:
				return fmt.Errorf("unknown event dimension: %s", event.EventID)
			}
			if event.Supersedes != "" {
				prior, ok := seen[event.Supersedes]
				if !ok || prior.Dimension != event.Dimension || prior.IntervalRef != event.IntervalRef {
					return fmt.Errorf("invalid supersession lineage: %s", event.EventID)
				}
			}
			seen[event.EventID] = event
		}
		logs[log.OwnerRef] = log
	}
	if len(s.Frontiers) == 0 { return fmt.Errorf("scenario has no frontier variants") }
	for label, frontier := range s.Frontiers {
		if label == "" || frontier.FrontierRef == "" || !digest(frontier.FrontierDigest) ||
			frontier.RegistryRef == "" || frontier.RegistryEpoch == "" || frontier.UniverseProfileRef == "" {
			return fmt.Errorf("frontier %s lacks identity/epoch commitment", label)
		}
		seenOwners := make(map[string]bool)
		for _, cut := range frontier.Owners {
			candidate, ok := candidates[cut.OwnerRef]
			log, logOK := logs[cut.OwnerRef]
			if !ok || !required[cut.OwnerRef] || !logOK || seenOwners[cut.OwnerRef] ||
				candidate.HistoryRef != cut.HistoryRef || cut.HistoryRef != log.HistoryRef ||
				!digest(cut.LogRootDigest) || cut.SequenceCut > uint64(len(log.Events)) {
				return fmt.Errorf("invalid frontier owner binding in %s: %s", label, cut.OwnerRef)
			}
			seenOwners[cut.OwnerRef] = true
		}
		if _, ok := s.Expected.FrontierResults[label]; !ok { return fmt.Errorf("missing expected result for frontier %s", label) }
	}
	for label := range s.Expected.FrontierResults {
		if _, ok := s.Frontiers[label]; !ok { return fmt.Errorf("expected result for undeclared frontier %s", label) }
	}
	return nil
}

func digest(s string) bool {
	if len(s) != 64 || strings.ToLower(s) != s { return false }
	for _, r := range s { if !((r >= '0' && r <= '9') || (r >= 'a' && r <= 'f')) { return false } }
	return true
}

func TestCorpusManifestAndStrictScenarioValidation(t *testing.T) {
	scenarios, manifest := loadCorpus(t)
	if manifest.CorpusVersion != "MYCELIX-TEMPORAL-ADS-CORPUS-V0.1" || manifest.ScenarioCount != len(scenarios) || manifest.ScenarioCount != len(manifest.ScenarioIDs) {
		t.Fatalf("manifest count/version mismatch: %+v scenarios=%d", manifest, len(scenarios))
	}
	seen := make(map[string]bool)
	for _, s := range scenarios {
		if err := validateScenario(s); err != nil { t.Fatalf("%s: %v", s.ScenarioID, err) }
		if seen[s.ScenarioID] { t.Fatalf("duplicate scenario ID: %s", s.ScenarioID) }
		seen[s.ScenarioID] = true
	}
	for _, id := range manifest.ScenarioIDs { if !seen[id] { t.Fatalf("manifest scenario missing: %s", id) } }
}

func TestFrozenCorpusMatchesIndependentReferenceOracles(t *testing.T) {
	scenarios, _ := loadCorpus(t)
	for _, s := range scenarios {
		t.Run(s.ScenarioID, func(t *testing.T) {
			scope := hindsight.Scope{Verifier:s.Scope.VerifierRef,Policy:s.Scope.PolicyEpochRef,Namespace:s.Scope.NamespaceRef,Subject:s.Scope.SubjectRef,Claim:s.Scope.ClaimID}
			owners:=make([]string,0)
			for _,candidate:=range s.Universe.Candidates { if candidate.Disposition=="Required" { owners=append(owners,candidate.OwnerRef) } }
			histories:=make(map[string][]hindsight.Event,len(s.OwnerLogs))
			for _,log:=range s.OwnerLogs {
				events:=make([]hindsight.Event,0,len(log.Events))
				for _,event:=range log.Events {
					dim:=hindsight.NonTemporal
					switch event.Dimension { case "VALID_FROM": dim=hindsight.ValidFrom; case "VALID_TO": dim=hindsight.ValidTo }
					value:=int64(0); if event.Value!=nil { value=*event.Value }
					events=append(events,hindsight.Event{Owner:log.OwnerRef,OwnerHistoryRef:log.HistoryRef,Sequence:event.Sequence,ID:event.EventID,Namespace:s.Scope.NamespaceRef,Subject:s.Scope.SubjectRef,Claim:s.Scope.ClaimID,IntervalRef:event.IntervalRef,Dimension:dim,Value:value,ExplicitOpen:event.ExplicitOpen,Supersedes:event.Supersedes})
				}
				histories[log.OwnerRef]=events
			}
			for label,frontier:=range s.Frontiers {
				cut:=make(hindsight.Cut,len(frontier.Owners))
				for _,owner:=range frontier.Owners { cut[owner.OwnerRef]=hindsight.OwnerCut{OwnerHistoryRef:owner.HistoryRef,Sequence:owner.SequenceCut} }
				result:=hindsight.Evaluate(owners,histories,cut,scope,s.Scope.ValidAt.Seconds)
				expected:=s.Expected.FrontierResults[label]
				if result.Disposition!=expected.Disposition { t.Fatalf("frontier %s disposition=%s want=%s reason=%s",label,result.Disposition,expected.Disposition,result.Reason) }
				actual:=make([]string,0,len(result.Matches)); for _,match:=range result.Matches { actual=append(actual,match.IntervalRef) }; sort.Strings(actual)
				wanted:=append([]string(nil),expected.MatchingIntervalRefs...); sort.Strings(wanted)
				if strings.Join(actual,"\x00")!=strings.Join(wanted,"\x00") { t.Fatalf("frontier %s matches=%v want=%v",label,actual,wanted) }
			}

			scopeDigest,err:=scopeclosure.ScopeDigest(scopeclosure.QueryScope{
				VerifierRef:s.Scope.VerifierRef,PolicyEpochRef:s.Scope.PolicyEpochRef,NamespaceRef:s.Scope.NamespaceRef,
				SubjectRef:s.Scope.SubjectRef,ClaimID:s.Scope.ClaimID,QueryProfileRef:s.Scope.QueryProfileRef,
				ValidAt:scopeclosure.Instant{UnixSeconds:s.Scope.ValidAt.Seconds,Nanoseconds:s.Scope.ValidAt.Nanos},
			})
			if err!=nil { t.Fatal(err) }
			candidates:=make([]scopeclosure.OwnerCandidate,0,len(s.Universe.Candidates))
			for _,candidate:=range s.Universe.Candidates { candidates=append(candidates,scopeclosure.OwnerCandidate{
				OwnerRef:candidate.OwnerRef,HistoryRef:candidate.HistoryRef,Disposition:scopeclosure.CandidateDisposition(candidate.Disposition),
				ExclusionCode:candidate.ExclusionCode,ExclusionProof:candidate.ExclusionProofDigest,
			}) }
			root,err:=scopeclosure.CandidateRoot(candidates); if err!=nil { t.Fatal(err) }
			universe:=scopeclosure.UniverseSnapshot{RegistryRef:s.Universe.RegistryRef,RegistryEpoch:s.Universe.RegistryEpoch,ProfileRef:s.Universe.ProfileRef,ScopeDigest:scopeDigest,SourceProofDigest:s.Universe.SourceProofDigest,Candidates:candidates}
			manifest:=scopeclosure.ScopeClosureManifest{RegistryRef:s.Universe.RegistryRef,RegistryEpoch:s.Universe.RegistryEpoch,ProfileRef:s.Universe.ProfileRef,ScopeDigest:scopeDigest,CandidateRoot:root,SourceProofDigest:s.Universe.SourceProofDigest,Candidates:candidates}
			label:="F0"; frontier,ok:=s.Frontiers[label]; if !ok { t.Fatalf("scenario requires F0 frontier for roster oracle") }
			frontierOwners:=make([]scopeclosure.OwnerFrontierBinding,0,len(frontier.Owners))
			for _,owner:=range frontier.Owners { frontierOwners=append(frontierOwners,scopeclosure.OwnerFrontierBinding{
				OwnerRef:owner.OwnerRef,HistoryRef:owner.HistoryRef,SequenceCut:owner.SequenceCut,TreeSize:owner.SequenceCut,LogRootDigest:owner.LogRootDigest,
			})}
			frontierSnapshot:=scopeclosure.FrontierSnapshot{
				VerifierRef:s.Scope.VerifierRef,PolicyEpochRef:s.Scope.PolicyEpochRef,
				RegistryRef:frontier.RegistryRef,RegistryEpoch:frontier.RegistryEpoch,UniverseProfileRef:frontier.UniverseProfileRef,
				ScopeDigest:scopeDigest,FrontierDigest:frontier.FrontierDigest,Owners:frontierOwners,
			}
			closure:=scopeclosure.Reconcile(scopeclosure.QueryScope{
				VerifierRef:s.Scope.VerifierRef,PolicyEpochRef:s.Scope.PolicyEpochRef,NamespaceRef:s.Scope.NamespaceRef,
				SubjectRef:s.Scope.SubjectRef,ClaimID:s.Scope.ClaimID,QueryProfileRef:s.Scope.QueryProfileRef,
				ValidAt:scopeclosure.Instant{UnixSeconds:s.Scope.ValidAt.Seconds,Nanoseconds:s.Scope.ValidAt.Nanos},
			},manifest,universe,frontierSnapshot)
			if string(closure.Disposition)!=s.Expected.RosterDisposition {
				t.Fatalf("roster disposition=%s want=%s reason=%s",closure.Disposition,s.Expected.RosterDisposition,closure.Reason)
			}
		})
	}
}
