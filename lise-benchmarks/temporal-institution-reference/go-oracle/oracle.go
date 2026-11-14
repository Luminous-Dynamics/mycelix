// Package temporaloracle independently replays the LISE temporal-institution
// trace contract using only the Go standard library.
package temporaloracle

import (
	"fmt"
	"math"
)

type Disposition string

const (
	Permit            Disposition = "permit"
	Deny              Disposition = "deny"
	Hold              Disposition = "hold"
	Indeterminate     Disposition = "indeterminate"
	ChallengeRequired Disposition = "challenge_required"
)

type EvidenceStatus string

const (
	EvidenceNotProvided EvidenceStatus = "not_provided"
	EvidenceValid       EvidenceStatus = "valid"
	EvidenceInvalid     EvidenceStatus = "invalid"
)

type ServiceMode string

const (
	ServiceNormal     ServiceMode = "normal"
	ServiceDegradedRO ServiceMode = "degraded_read_only"
	ServiceOffline    ServiceMode = "offline"
)

type State struct {
	CoalitionControlMask               uint8
	ControlInventoryComplete           bool
	FounderAvailable                   bool
	RecoveryTriggerVerified            bool
	RecoveryQuorumMet                  bool
	OldSessionsFenced                  bool
	ServiceMode                        ServiceMode
	HighImpactEnabled                  bool
	CurrentConstitutionEpoch           uint32
	CredentialEpoch                    uint32
	OldAdministratorRestored           bool
	EmergencyAuthorityActive           bool
	EmergencyAuthorityExpiresAt        *uint32
	Now                                uint32
	OversightRestorationAttempted      bool
	OversightRestorationAuthority      EvidenceStatus
	OversightRestorationIndependence   EvidenceStatus
	OversightRestorationQuorum         EvidenceStatus
	OversightRestorationAuthorityEpoch *uint32
}

type Finding struct {
	Disposition Disposition `json:"disposition"`
	ReasonCode  string      `json:"reason_code"`
}

const (
	capAppointments uint8 = 1 << iota
	capAgenda
	capTreasury
	capCredentials
	capExecution
	capOversight
)

const criticalForOversight = capAppointments | capTreasury | capCredentials | capExecution

func DefaultState() State {
	return State{
		ControlInventoryComplete:           true,
		FounderAvailable:                   true,
		ServiceMode:                         ServiceNormal,
		HighImpactEnabled:                   true,
		CurrentConstitutionEpoch:            1,
		CredentialEpoch:                     1,
		OversightRestorationAuthority:       EvidenceNotProvided,
		OversightRestorationIndependence:    EvidenceNotProvided,
		OversightRestorationQuorum:          EvidenceNotProvided,
	}
}

func finding(disposition Disposition, reason string) Finding {
	return Finding{Disposition: disposition, ReasonCode: reason}
}

// Assess evaluates the finite declared properties. Known hard violations
// take precedence over missing-evidence dispositions.
func Assess(s State) Finding {
	if s.ServiceMode == ServiceDegradedRO && s.HighImpactEnabled {
		return finding(Deny, "DEGRADED_MODE_MUST_KEEP_HIGH_IMPACT_DISABLED")
	}
	if s.ServiceMode == ServiceDegradedRO && !s.OldSessionsFenced {
		return finding(Deny, "DEGRADED_MODE_REQUIRES_OLD_SESSION_FENCING")
	}
	if !s.FounderAvailable && s.HighImpactEnabled &&
		!(s.RecoveryTriggerVerified && s.RecoveryQuorumMet && s.OldSessionsFenced &&
			s.CredentialEpoch == s.CurrentConstitutionEpoch) {
		return finding(Deny, "HIGH_IMPACT_RECOVERY_REQUIRES_TRIGGER_QUORUM_FENCING_AND_REVALIDATION")
	}
	if s.OldAdministratorRestored && s.CredentialEpoch < s.CurrentConstitutionEpoch {
		return finding(Deny, "STALE_ADMINISTRATOR_PRIVILEGE_RESTORED_AFTER_EPOCH_CHANGE")
	}
	if s.EmergencyAuthorityActive {
		if s.EmergencyAuthorityExpiresAt == nil {
			return finding(Indeterminate, "EMERGENCY_AUTHORITY_EXPIRY_UNSPECIFIED")
		}
		if s.Now >= *s.EmergencyAuthorityExpiresAt {
			return finding(Deny, "EXPIRED_EMERGENCY_AUTHORITY_REMAINS_ACTIVE")
		}
	}
	if s.OversightRestorationAttempted {
		statuses := [...]EvidenceStatus{
			s.OversightRestorationAuthority,
			s.OversightRestorationIndependence,
			s.OversightRestorationQuorum,
		}
		for _, status := range statuses {
			if status == EvidenceInvalid {
				return finding(Deny, "OVERSIGHT_RESTORATION_EVIDENCE_INVALID")
			}
		}
		if s.OversightRestorationAuthorityEpoch == nil {
			return finding(Indeterminate, "OVERSIGHT_RESTORATION_EVIDENCE_INCOMPLETE")
		}
		if *s.OversightRestorationAuthorityEpoch != s.CurrentConstitutionEpoch {
			return finding(Deny, "OVERSIGHT_RESTORATION_AUTHORITY_EPOCH_MISMATCH")
		}
		for _, status := range statuses {
			if status == EvidenceNotProvided {
				return finding(Indeterminate, "OVERSIGHT_RESTORATION_EVIDENCE_INCOMPLETE")
			}
		}
	}
	if !s.ControlInventoryComplete {
		return finding(Indeterminate, "CONTROL_INVENTORY_INCOMPLETE")
	}
	oversightCaptured := s.CoalitionControlMask&capOversight != 0
	criticalCaptured := s.CoalitionControlMask&criticalForOversight != 0
	if oversightCaptured && criticalCaptured {
		return finding(ChallengeRequired, "OVERSIGHT_SHARES_CONTROL_WITH_APPOINTMENT_RESOURCE_OR_CREDENTIAL_COALITION")
	}
	return finding(Permit, "DECLARED_TEMPORAL_PROPERTIES_HOLD")
}

// Apply deliberately permits unsafe transitions so replay can expose them.
func Apply(s State, action string) (State, error) {
	n := s
	switch action {
	case "capture_appointments":
		n.CoalitionControlMask |= capAppointments
	case "capture_agenda":
		n.CoalitionControlMask |= capAgenda
	case "capture_treasury":
		n.CoalitionControlMask |= capTreasury
	case "capture_credentials":
		n.CoalitionControlMask |= capCredentials
	case "capture_execution":
		n.CoalitionControlMask |= capExecution
	case "capture_oversight":
		n.CoalitionControlMask |= capOversight
	case "mark_inventory_incomplete":
		n.ControlInventoryComplete = false
	case "restore_independent_oversight":
		n.OversightRestorationAttempted = true
		if n.OversightRestorationAuthority == EvidenceValid &&
			n.OversightRestorationIndependence == EvidenceValid &&
			n.OversightRestorationQuorum == EvidenceValid &&
			n.OversightRestorationAuthorityEpoch != nil &&
			*n.OversightRestorationAuthorityEpoch == n.CurrentConstitutionEpoch {
			n.CoalitionControlMask &^= capOversight
		}
	case "founder_unavailable":
		n.FounderAvailable = false
	case "verify_recovery_trigger":
		if !n.FounderAvailable {
			n.RecoveryTriggerVerified = true
		}
	case "reach_recovery_quorum":
		if n.RecoveryTriggerVerified && n.ControlInventoryComplete {
			n.RecoveryQuorumMet = true
		}
	case "fence_old_sessions":
		n.OldSessionsFenced = true
	case "restore_degraded_read_only":
		n.ServiceMode = ServiceDegradedRO
		n.HighImpactEnabled = false
	case "restore_old_administrator":
		n.OldAdministratorRestored = true
	case "change_constitution":
		if n.CurrentConstitutionEpoch < math.MaxUint32 {
			n.CurrentConstitutionEpoch++
		}
	case "revalidate_credentials":
		n.CredentialEpoch = n.CurrentConstitutionEpoch
		n.OldAdministratorRestored = false
	case "enable_high_impact":
		n.ServiceMode = ServiceNormal
		n.HighImpactEnabled = true
	case "disable_high_impact":
		n.HighImpactEnabled = false
	case "advance_time":
		if n.Now < math.MaxUint32 {
			n.Now++
		}
	case "grant_emergency_authority":
		n.EmergencyAuthorityActive = true
		expiry := n.Now
		if expiry <= math.MaxUint32-2 {
			expiry += 2
		} else {
			expiry = math.MaxUint32
		}
		n.EmergencyAuthorityExpiresAt = &expiry
	case "revoke_emergency_authority":
		n.EmergencyAuthorityActive = false
		n.EmergencyAuthorityExpiresAt = nil
	default:
		return s, fmt.Errorf("unknown action %q", action)
	}
	return n, nil
}

func Replay(actions []string) (State, Finding, error) {
	s := DefaultState()
	for i, action := range actions {
		next, err := Apply(s, action)
		if err != nil {
			return s, Finding{}, fmt.Errorf("action %d: %w", i, err)
		}
		s = next
	}
	return s, Assess(s), nil
}
