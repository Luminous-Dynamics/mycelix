package oracle

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

type expectedKAT struct {
	Status             Status   `json:"status"`
	ReasonContains     string   `json:"reason_contains"`
	BeforeExpectedLoss *float64 `json:"before_expected_loss"`
	AfterExpectedLoss  *float64 `json:"after_expected_loss"`
	EVSI               *float64 `json:"evsi"`
	EIGBits            *float64 `json:"eig_bits"`
	NormalizedCost     *float64 `json:"normalized_cost"`
	NetDecisionValue   *float64 `json:"net_decision_value"`
}

type fixtureKAT struct {
	Case        Case        `json:"case"`
	Observation Observation `json:"observation"`
	Expected    expectedKAT `json:"expected"`
}

type fixtureFile struct {
	SchemaID string       `json:"schema_id"`
	Cases    []fixtureKAT `json:"cases"`
}

func near(t *testing.T, name string, got float64, expected *float64) {
	t.Helper()
	if expected == nil {
		return
	}
	if math.Abs(got-*expected) > 1e-8 {
		t.Fatalf("%s = %.12f, expected %.12f", name, got, *expected)
	}
}

func TestFrozenEVSIKnownAnswers(t *testing.T) {
	path := filepath.Join("..", "fixtures", "evsi-known-answer-v0.1.json")
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var fixture fixtureFile
	if err := json.Unmarshal(data, &fixture); err != nil {
		t.Fatal(err)
	}
	if fixture.SchemaID != "mycelix.craft-evsi-known-answer-v0.1" {
		t.Fatalf("unexpected schema %q", fixture.SchemaID)
	}
	if len(fixture.Cases) != 12 {
		t.Fatalf("expected 12 frozen cases, got %d", len(fixture.Cases))
	}
	seen := map[string]bool{}
	for _, item := range fixture.Cases {
		item := item
		t.Run(item.Case.ID, func(t *testing.T) {
			if seen[item.Case.ID] {
				t.Fatalf("duplicate case id %s", item.Case.ID)
			}
			seen[item.Case.ID] = true
			report, err := Evaluate(item.Case, item.Observation)
			if err != nil {
				t.Fatal(err)
			}
			if report.Status != item.Expected.Status {
				t.Fatalf("status = %s, expected %s; reason=%s", report.Status, item.Expected.Status, report.Reason)
			}
			if item.Expected.ReasonContains != "" && !strings.Contains(report.Reason, item.Expected.ReasonContains) {
				t.Fatalf("reason %q does not contain %q", report.Reason, item.Expected.ReasonContains)
			}
			near(t, "before_expected_loss", report.BeforeExpectedLoss, item.Expected.BeforeExpectedLoss)
			near(t, "after_expected_loss", report.AfterExpectedLoss, item.Expected.AfterExpectedLoss)
			near(t, "evsi", report.EVSI, item.Expected.EVSI)
			near(t, "eig_bits", report.EIGBits, item.Expected.EIGBits)
			near(t, "normalized_cost", report.NormalizedCost, item.Expected.NormalizedCost)
			near(t, "net_decision_value", report.NetDecisionValue, item.Expected.NetDecisionValue)
		})
	}
}

func TestMandatoryEvidenceDominatesOptionalValue(t *testing.T) {
	var fixture fixtureFile
	data, err := os.ReadFile(filepath.Join("..", "fixtures", "evsi-known-answer-v0.1.json"))
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &fixture); err != nil {
		t.Fatal(err)
	}
	for _, item := range fixture.Cases {
		if item.Case.ID != "EVSI-006" {
			continue
		}
		report, err := Evaluate(item.Case, item.Observation)
		if err != nil {
			t.Fatal(err)
		}
		if report.Status != StatusAcquireMandatory {
			t.Fatalf("mandatory evidence was not preserved: %s", report.Status)
		}
		return
	}
	t.Fatal("EVSI-006 missing")
}

func TestMalformedLikelihoodDistributionFailsClosed(t *testing.T) {
	c := Case{ID: "bad", States: []State{{ID: "s", Probability: 1}}, Actions: []Action{{ID: "hold", Loss: map[string]float64{"s": 1}}}, AllowedActionsBefore: []string{"hold"}, CostProfile: CostProfile{ID: "cost-v1"}}
	o := Observation{ID: "bad-observation", Available: true, InDomain: true, AdmissibleForDecision: true, ConditionedOnCurrentEvidence: true, LikelihoodByState: map[string][]Likelihood{"s": {{ResultID: "x", Given: 0.4}}}, AllowedActionsByResult: map[string][]string{"x": {"hold"}}}
	if _, err := Evaluate(c, o); err == nil {
		t.Fatal("non-normalized observation likelihood was accepted")
	}
}

func TestOutOfDomainAndUnknownLineageAbstain(t *testing.T) {
	var fixture fixtureFile
	data, err := os.ReadFile(filepath.Join("..", "fixtures", "evsi-known-answer-v0.1.json"))
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &fixture); err != nil {
		t.Fatal(err)
	}
	for _, item := range fixture.Cases {
		if item.Case.ID != "EVSI-005" && item.Case.ID != "EVSI-008" && item.Case.ID != "EVSI-009" {
			continue
		}
		report, err := Evaluate(item.Case, item.Observation)
		if err != nil {
			t.Fatal(err)
		}
		if report.Status != StatusAbstain {
			t.Fatalf("%s failed to abstain: %s", item.Case.ID, report.Status)
		}
	}
}
