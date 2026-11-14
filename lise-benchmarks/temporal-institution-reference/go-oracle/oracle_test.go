package temporaloracle

import (
	"bytes"
	"encoding/json"
	"io"
	"os"
	"testing"
)

type traceCase struct {
	ID      string   `json:"id"`
	Title   string   `json:"title"`
	Actions []string `json:"actions"`
}
type traceSuite struct {
	SchemaVersion string      `json:"schema_version"`
	SuiteID       string      `json:"suite_id"`
	Status        string      `json:"status"`
	CaseCount     int         `json:"case_count"`
	Cases         []traceCase `json:"cases"`
}
type oracleRow struct {
	CaseID              string      `json:"case_id"`
	ExpectedDisposition Disposition `json:"expected_disposition"`
	ReasonCode          string      `json:"reason_code"`
}
type oracleSuite struct {
	SchemaVersion string      `json:"schema_version"`
	SuiteID       string      `json:"suite_id"`
	Status        string      `json:"status"`
	CaseCount     int         `json:"case_count"`
	Rows          []oracleRow `json:"rows"`
}

func decodeStrict[T any](t *testing.T, path string) T {
	t.Helper()
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read %s: %v", path, err)
	}
	var out T
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&out); err != nil {
		t.Fatalf("decode %s: %v", path, err)
	}
	var extra any
	if err := dec.Decode(&extra); err != io.EOF {
		t.Fatalf("%s contains trailing JSON data", path)
	}
	return out
}

func TestFrozenTraceSuiteAgainstSeparateOracle(t *testing.T) {
	suite := decodeStrict[traceSuite](t, "../fixtures/scenarios.json")
	oracle := decodeStrict[oracleSuite](t, "../fixtures/oracle.json")
	if suite.SchemaVersion != "lise.temporal-institution-scenarios.v0.1" ||
		oracle.SchemaVersion != "lise.temporal-institution-oracle.v0.1" {
		t.Fatal("unexpected fixture schema version")
	}
	if suite.SuiteID == "" || suite.SuiteID != oracle.SuiteID {
		t.Fatalf("suite identity mismatch: %q vs %q", suite.SuiteID, oracle.SuiteID)
	}
	if suite.CaseCount != len(suite.Cases) || oracle.CaseCount != len(oracle.Rows) ||
		len(suite.Cases) != len(oracle.Rows) {
		t.Fatalf("count mismatch: scenarios=%d/%d oracle=%d/%d",
			suite.CaseCount, len(suite.Cases), oracle.CaseCount, len(oracle.Rows))
	}
	rows := make(map[string]oracleRow, len(oracle.Rows))
	for _, row := range oracle.Rows {
		if row.CaseID == "" || row.ReasonCode == "" {
			t.Fatalf("oracle row has an empty key/reason: %+v", row)
		}
		if _, exists := rows[row.CaseID]; exists {
			t.Fatalf("duplicate oracle id %q", row.CaseID)
		}
		rows[row.CaseID] = row
	}
	seen := make(map[string]bool, len(suite.Cases))
	for _, tc := range suite.Cases {
		if tc.ID == "" || seen[tc.ID] {
			t.Fatalf("empty or duplicate scenario id %q", tc.ID)
		}
		seen[tc.ID] = true
		want, ok := rows[tc.ID]
		if !ok {
			t.Fatalf("scenario %s has no oracle row", tc.ID)
		}
		_, got, err := Replay(tc.Actions)
		if err != nil {
			t.Fatalf("scenario %s replay: %v", tc.ID, err)
		}
		if got.Disposition != want.ExpectedDisposition || got.ReasonCode != want.ReasonCode {
			t.Errorf("%s (%s): got %s/%s, want %s/%s",
				tc.ID, tc.Title, got.Disposition, got.ReasonCode,
				want.ExpectedDisposition, want.ReasonCode)
		}
	}
	for id := range rows {
		if !seen[id] {
			t.Errorf("oracle-only case %s", id)
		}
	}
}

func TestOversightRestorationRequiresCurrentAuthorityIndependenceAndQuorum(t *testing.T) {
	s := DefaultState()
	s.CoalitionControlMask = capTreasury | capOversight
	s.OversightRestorationAttempted = true
	if got := Assess(s); got.Disposition != Indeterminate ||
		got.ReasonCode != "OVERSIGHT_RESTORATION_EVIDENCE_INCOMPLETE" {
		t.Fatalf("missing evidence: got %+v", got)
	}
	s.OversightRestorationAuthority = EvidenceValid
	s.OversightRestorationIndependence = EvidenceValid
	s.OversightRestorationQuorum = EvidenceValid
	epoch := s.CurrentConstitutionEpoch
	s.OversightRestorationAuthorityEpoch = &epoch
	next, err := Apply(s, "restore_independent_oversight")
	if err != nil {
		t.Fatal(err)
	}
	if next.CoalitionControlMask&capOversight != 0 {
		t.Fatal("validly restored oversight remained in captured coalition")
	}
	if got := Assess(next); got.Disposition != Permit {
		t.Fatalf("valid restoration: got %+v", got)
	}
	stale := s
	staleEpoch := epoch - 1
	stale.OversightRestorationAuthorityEpoch = &staleEpoch
	if got := Assess(stale); got.Disposition != Deny ||
		got.ReasonCode != "OVERSIGHT_RESTORATION_AUTHORITY_EPOCH_MISMATCH" {
		t.Fatalf("stale epoch: got %+v", got)
	}
}

func TestInvalidOversightEvidenceIsDenied(t *testing.T) {
	s := DefaultState()
	s.OversightRestorationAttempted = true
	s.OversightRestorationAuthority = EvidenceInvalid
	s.OversightRestorationIndependence = EvidenceValid
	s.OversightRestorationQuorum = EvidenceValid
	epoch := s.CurrentConstitutionEpoch
	s.OversightRestorationAuthorityEpoch = &epoch
	got := Assess(s)
	if got.Disposition != Deny || got.ReasonCode != "OVERSIGHT_RESTORATION_EVIDENCE_INVALID" {
		t.Fatalf("got %+v", got)
	}
}

func TestUnknownEmergencyExpiryIsNotPermit(t *testing.T) {
	s := DefaultState()
	s.EmergencyAuthorityActive = true
	got := Assess(s)
	if got.Disposition != Indeterminate || got.ReasonCode != "EMERGENCY_AUTHORITY_EXPIRY_UNSPECIFIED" {
		t.Fatalf("got %+v", got)
	}
}

func TestHardViolationPrecedesIncompleteInventory(t *testing.T) {
	s := DefaultState()
	s.FounderAvailable = false
	s.ControlInventoryComplete = false
	got := Assess(s)
	if got.Disposition != Deny ||
		got.ReasonCode != "HIGH_IMPACT_RECOVERY_REQUIRES_TRIGGER_QUORUM_FENCING_AND_REVALIDATION" {
		t.Fatalf("got %+v", got)
	}
}

func TestUnknownActionFailsClosed(t *testing.T) {
	if _, _, err := Replay([]string{"capture_treasury", "invented_transition"}); err == nil {
		t.Fatal("unknown transition was accepted")
	}
}

func TestExpiryIsAnExclusiveUpperBound(t *testing.T) {
	s := DefaultState()
	s.EmergencyAuthorityActive = true
	expiry := uint32(4)
	s.EmergencyAuthorityExpiresAt = &expiry
	s.Now = 3
	if got := Assess(s); got.Disposition != Permit {
		t.Fatalf("before expiry: got %+v", got)
	}
	s.Now = 4
	if got := Assess(s); got.Disposition != Deny {
		t.Fatalf("at expiry: got %+v", got)
	}
}
