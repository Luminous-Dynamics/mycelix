package main

import (
	"encoding/json"
	"os"
	"reflect"
	"testing"
)

type challengePack struct {
	Schema          string          `json:"schema"`
	Version         string          `json:"version"`
	State           string          `json:"state"`
	ClaimCeiling    string          `json:"claim_ceiling"`
	EvaluatorTruth  string          `json:"evaluator_truth"`
	CaseCount       int             `json:"case_count"`
	Cases           []challengeCase `json:"cases"`
}

type challengeCase struct {
	ID                  string   `json:"id"`
	Mutation            string   `json:"mutation"`
	ExpectedDisposition string   `json:"expected_disposition"`
	ExpectedReasonCodes []string `json:"expected_reason_codes"`
}

func baseInput() Input {
	return Input{
		Contract: inputContract,
		Snapshot: InventorySnapshot{
			SnapshotID: "snapshot-dh-001",
			AsOf: 100,
			EvidenceFrontier: 120,
			Complete: true,
			ScopedSourceIDs: []string{"rx-a", "rx-b"},
			CoveredNodeIDs: []string{"clock-root", "provider-a", "provider-b", "rx-a", "rx-b"},
		},
		Query: DependencyQuery{
			SourceIDs: []string{"rx-a", "rx-b"},
			Domain: "clock",
			EventTime: 90,
			ExpectedSnapshotID: "snapshot-dh-001",
			ExpectedEvidenceFrontier: 120,
		},
		Nodes: []Node{
			{ID:"clock-root",Kind:"dependency"},
			{ID:"provider-a",Kind:"dependency"},
			{ID:"provider-b",Kind:"dependency"},
			{ID:"rx-a",Kind:"source"},
			{ID:"rx-b",Kind:"source"},
		},
		Hyperedges: []DependencyHyperedge{
			{ID:"edge-a",SubjectIDs:[]string{"rx-a"},DependencyID:"provider-a",Domain:"clock",ValidFrom:int64Ptr(0),RecordedAt:80,EvidenceRef:"evidence:a",State:"qualified"},
			{ID:"edge-b",SubjectIDs:[]string{"rx-b"},DependencyID:"provider-b",Domain:"clock",ValidFrom:int64Ptr(0),RecordedAt:80,EvidenceRef:"evidence:b",State:"qualified"},
			{ID:"shared-root",SubjectIDs:[]string{"provider-a","provider-b"},DependencyID:"clock-root",Domain:"clock",ValidFrom:int64Ptr(0),RecordedAt:80,EvidenceRef:"evidence:root",State:"qualified"},
		},
	}
}

func int64Ptr(value int64) *int64 { return &value }

func mutate(input *Input, mutation string) {
	switch mutation {
	case "transitive_shared_ancestor":
	case "direct_multi_subject_hyperedge":
		input.Hyperedges = input.Hyperedges[:2]
		input.Hyperedges = append(input.Hyperedges, DependencyHyperedge{
			ID:"one-clock-for-both",SubjectIDs:[]string{"rx-a","rx-b"},DependencyID:"clock-root",
			Domain:"clock",RecordedAt:80,EvidenceRef:"evidence:one-clock",State:"qualified",
		})
	case "shared_power_does_not_imply_clock":
		input.Nodes = append(input.Nodes, Node{ID:"power-root",Kind:"dependency"})
		sortNodes(input.Nodes)
		input.Snapshot.CoveredNodeIDs = append(input.Snapshot.CoveredNodeIDs,"power-root")
		sortStrings(input.Snapshot.CoveredNodeIDs)
		input.Hyperedges = []DependencyHyperedge{{
			ID:"power-only",SubjectIDs:[]string{"rx-a","rx-b"},DependencyID:"power-root",Domain:"power",
			ValidFrom:int64Ptr(0),RecordedAt:80,EvidenceRef:"evidence:power-only",State:"qualified",
		}}
		input.Query.Domain = "clock"
	case "shared_edge_expired_at_event_time":
		input.Hyperedges[2].ValidUntil = int64Ptr(50)
	case "stale_shared_edge":
		input.Hyperedges[2].State = "stale"
	case "source_missing_from_inventory_scope":
		input.Snapshot.ScopedSourceIDs = input.Snapshot.ScopedSourceIDs[:1]
	case "incomplete_inventory_snapshot":
		input.Snapshot.Complete = false
	case "source_missing_from_snapshot_coverage":
		input.Snapshot.CoveredNodeIDs = input.Snapshot.CoveredNodeIDs[:len(input.Snapshot.CoveredNodeIDs)-1]
	case "dependency_cycle":
		input.Hyperedges = append(input.Hyperedges, DependencyHyperedge{
			ID:"cycle",SubjectIDs:[]string{"clock-root"},DependencyID:"provider-a",Domain:"clock",
			ValidFrom:int64Ptr(0),RecordedAt:80,EvidenceRef:"evidence:cycle",State:"qualified",
		})
	case "tampered_closure_report":
	case "noncanonical_source_order":
		input.Query.SourceIDs = []string{"rx-b","rx-a"}
	case "complete_inventory_no_common_ancestor":
		input.Hyperedges = input.Hyperedges[:2]
	default:
		panic("unknown challenge mutation: "+mutation)
	}
}

func sortNodes(nodes []Node) {
	for i:=0;i<len(nodes);i++ {
		for j:=i+1;j<len(nodes);j++ {
			if nodes[j].ID<nodes[i].ID { nodes[i],nodes[j]=nodes[j],nodes[i] }
		}
	}
}
func sortStrings(values []string) {
	for i:=0;i<len(values);i++ {
		for j:=i+1;j<len(values);j++ {
			if values[j]<values[i] { values[i],values[j]=values[j],values[i] }
		}
	}
}

func TestPublicChallengePack(t *testing.T) {
	data, err := os.ReadFile("../emsr-dependency-hypergraph/fixtures/dependency-hypergraph-challenges-0.1.json")
	if err != nil { t.Fatal(err) }
	var pack challengePack
	if err := json.Unmarshal(data,&pack); err != nil { t.Fatal(err) }
	if pack.Schema!="emsr-dependency-hypergraph-challenge-pack/0.1" || pack.Version!="0.1.0" ||
		pack.State!="FROZEN_PUBLIC_SYNTHETIC_CONFORMANCE" || pack.CaseCount!=len(pack.Cases) ||
		pack.ClaimCeiling=="" || pack.EvaluatorTruth=="" {
		t.Fatal("challenge pack contract is not frozen or complete")
	}
	expectedIDs:=[]string{"DH-001","DH-002","DH-003","DH-004","DH-005","DH-006","DH-007","DH-008","DH-009","DH-010","DH-011","DH-012"}
	actualIDs:=make([]string,0,len(pack.Cases))
	for _,challenge:=range pack.Cases { actualIDs=append(actualIDs,challenge.ID) }
	if !reflect.DeepEqual(actualIDs,expectedIDs) { t.Fatalf("challenge discovery mismatch: %v",actualIDs) }

	for _,challenge:=range pack.Cases {
		t.Run(challenge.ID,func(t *testing.T) {
			input:=baseInput()
			mutate(&input,challenge.Mutation)
			inputBytes,err:=json.Marshal(input)
			if err!=nil { t.Fatal(err) }
			expected:=referenceReport(input,sha256Hex(inputBytes))
			if challenge.ExpectedDisposition=="VERIFIER_REJECT" {
				expected.CommonDependencyIDs=[]string{}
				reportBytes,_:=json.Marshal(expected)
				result:=verify(inputBytes,reportBytes)
				if result.Status!="rejected" { t.Fatalf("tampered report unexpectedly verified: %+v",result) }
				return
			}
			if expected.Disposition!=challenge.ExpectedDisposition {
				t.Fatalf("disposition %s, want %s; reasons=%v",expected.Disposition,challenge.ExpectedDisposition,expected.ReasonCodes)
			}
			for _,reason:=range challenge.ExpectedReasonCodes {
				if !contains(expected.ReasonCodes,reason) { t.Fatalf("missing reason %q in %v",reason,expected.ReasonCodes) }
			}
			reportBytes,err:=json.Marshal(expected)
			if err!=nil { t.Fatal(err) }
			result:=verify(inputBytes,reportBytes)
			if result.Status!="verified" { t.Fatalf("valid report did not verify: %+v",result) }
		})
	}
}

func TestTamperedInputByteBindingRejected(t *testing.T) {
	input:=baseInput()
	inputBytes,_:=json.Marshal(input)
	expected:=referenceReport(input,sha256Hex(inputBytes))
	reportBytes,_:=json.Marshal(expected)
	mutated:=append([]byte(nil),inputBytes...)
	mutated=append(mutated,' ')
	result:=verify(mutated,reportBytes)
	if result.Status!="rejected" { t.Fatal("report with stale input digest was accepted") }
}

func TestInvalidAuthorityAndClaimCeilingRejected(t *testing.T) {
	input:=baseInput()
	inputBytes,_:=json.Marshal(input)
	expected:=referenceReport(input,sha256Hex(inputBytes))
	expected.Authority="production_operation_authority"
	reportBytes,_:=json.Marshal(expected)
	result:=verify(inputBytes,reportBytes)
	if result.Status!="rejected" { t.Fatal("authority escalation was accepted") }
}
