package scopeclosure

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"strings"
)

type SourceCompletionDisposition string

const (
	SourceDeclaredComplete   SourceCompletionDisposition = "SourceDeclaredComplete"
	SourceDeclaredIncomplete SourceCompletionDisposition = "SourceDeclaredIncomplete"
	SourceCompletionUnknown  SourceCompletionDisposition = "SourceCompletionUnknown"
)

type EnumerationManifestV1 struct {
	SourceRef, SnapshotRef, RegistryRef, RegistryEpoch string
	ProfileRef, ScopeDigest, PredicateDigest           string
	PageCount                                          uint64
	FirstPageDigest, TerminalPageDigest                string
	CandidateRoot, SourceCompletionReceiptDigest       string
	CompletionDisposition                              SourceCompletionDisposition
}

type EnumerationPageV1 struct {
	SourceRef, SnapshotRef, RegistryRef, RegistryEpoch string
	ProfileRef, ScopeDigest, PredicateDigest           string
	PageIndex                                          uint64
	RequestCursorDigest, NextCursorDigest              string
	Candidates                                         []OwnerCandidate
	Terminal                                           bool
	PageDigest                                         string
}

type EnumerationChainDisposition string

const (
	EnumerationChainIndeterminate EnumerationChainDisposition = "Indeterminate"
	EnumerationChainWellFormed    EnumerationChainDisposition = "EnumerationPageChainWellFormed"
)

type EnumerationPageChainResultV1 struct {
	Disposition                          EnumerationChainDisposition
	Reason                               string
	SourceRef, SnapshotRef               string
	RegistryRef, RegistryEpoch           string
	ProfileRef, ScopeDigest              string
	PredicateDigest, CandidateRoot       string
	ExpectedPageCount, ObservedPageCount uint64
	SourceCompletionReceiptDigest       string
	CompletionDisposition               SourceCompletionDisposition
	ManifestDigest                      string
	PageChainDigest, CertificateDigest  string
	PageChainAuthentication             string
	SourceCompletenessAuthentication    string
}

type enumerationField struct{ label, value string }

func validateEnumerationIdentity(fields []enumerationField) error {
	for _, field := range fields {
		if strings.TrimSpace(field.value) == "" {
			return fmt.Errorf("empty %s", field.label)
		}
	}
	return nil
}

// EnumerationPageDigestV1 commits to the page's exact source/snapshot identity,
// query scope, pagination transition, terminal marker and canonical candidates.
// It is a commitment only; it does not authenticate the page's source.
func EnumerationPageDigestV1(p EnumerationPageV1) (string, error) {
	if err := validateEnumerationIdentity([]enumerationField{
		{"source", p.SourceRef}, {"snapshot", p.SnapshotRef},
		{"registry", p.RegistryRef}, {"registry epoch", p.RegistryEpoch},
		{"profile", p.ProfileRef},
	}); err != nil {
		return "", err
	}
	if !validDigest(p.ScopeDigest) || !validDigest(p.PredicateDigest) {
		return "", errors.New("scope or enumeration-predicate digest is malformed")
	}
	if p.PageIndex == 0 {
		if p.RequestCursorDigest != "" {
			return "", errors.New("first page must not have a request cursor")
		}
	} else if !validDigest(p.RequestCursorDigest) {
		return "", errors.New("non-first page requires a request cursor digest")
	}
	if p.Terminal {
		if p.NextCursorDigest != "" {
			return "", errors.New("terminal page must not expose a next cursor")
		}
	} else if !validDigest(p.NextCursorDigest) {
		return "", errors.New("non-terminal page requires a next cursor digest")
	}
	root, err := CandidateRoot(p.Candidates)
	if err != nil {
		return "", fmt.Errorf("invalid page candidate inventory: %w", err)
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-OWNER-ENUMERATION-PAGE-V1")
	for _, value := range []string{
		p.SourceRef, p.SnapshotRef, p.RegistryRef, p.RegistryEpoch,
		p.ProfileRef, p.ScopeDigest, p.PredicateDigest,
	} {
		writeString(&b, value)
	}
	_ = binary.Write(&b, binary.BigEndian, p.PageIndex)
	writeString(&b, p.RequestCursorDigest)
	writeString(&b, p.NextCursorDigest)
	writeString(&b, root)
	if p.Terminal { b.WriteByte(1) } else { b.WriteByte(0) }
	d := sha256.Sum256(b.Bytes())
	return hex.EncodeToString(d[:]), nil
}

// EnumerationManifestDigestV1 commits every manifest field, including the
// source's completion claim and exact completion-receipt reference. It does
// not verify that receipt or establish that the source was actually complete.
func EnumerationManifestDigestV1(m EnumerationManifestV1) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-OWNER-ENUMERATION-MANIFEST-V1")
	for _, value := range []string{
		m.SourceRef, m.SnapshotRef, m.RegistryRef, m.RegistryEpoch,
		m.ProfileRef, m.ScopeDigest, m.PredicateDigest,
	} {
		writeString(&b, value)
	}
	_ = binary.Write(&b, binary.BigEndian, m.PageCount)
	writeString(&b, m.FirstPageDigest)
	writeString(&b, m.TerminalPageDigest)
	writeString(&b, m.CandidateRoot)
	writeString(&b, m.SourceCompletionReceiptDigest)
	writeString(&b, string(m.CompletionDisposition))
	d := sha256.Sum256(b.Bytes())
	return hex.EncodeToString(d[:])
}

// VerifyEnumerationPageChainV1 checks page-chain structure and internal
// consistency only. It does not authenticate the enumerator, snapshot,
// completion receipt, registry coverage, pagination authority or source truth.
// Its strongest positive result is EnumerationPageChainWellFormed.
func VerifyEnumerationPageChainV1(m EnumerationManifestV1, pages []EnumerationPageV1) EnumerationPageChainResultV1 {
	result := EnumerationPageChainResultV1{
		Disposition: EnumerationChainIndeterminate,
		SourceRef: m.SourceRef, SnapshotRef: m.SnapshotRef,
		RegistryRef: m.RegistryRef, RegistryEpoch: m.RegistryEpoch,
		ProfileRef: m.ProfileRef, ScopeDigest: m.ScopeDigest,
		PredicateDigest: m.PredicateDigest, CandidateRoot: m.CandidateRoot,
		ExpectedPageCount: m.PageCount, ObservedPageCount: uint64(len(pages)),
		SourceCompletionReceiptDigest: m.SourceCompletionReceiptDigest,
		CompletionDisposition: m.CompletionDisposition,
		ManifestDigest: EnumerationManifestDigestV1(m),
		PageChainAuthentication: "NotEvaluated",
		SourceCompletenessAuthentication: "NotEvaluated",
	}
	fail := func(reason string) EnumerationPageChainResultV1 {
		result.Reason = reason
		return finalizeEnumerationPageChainV1(result)
	}

	if err := validateEnumerationIdentity([]enumerationField{
		{"manifest source", m.SourceRef}, {"manifest snapshot", m.SnapshotRef},
		{"manifest registry", m.RegistryRef}, {"manifest registry epoch", m.RegistryEpoch},
		{"manifest profile", m.ProfileRef},
	}); err != nil {
		return fail(err.Error())
	}
	if !validDigest(m.ScopeDigest) || !validDigest(m.PredicateDigest) ||
		!validDigest(m.CandidateRoot) || !validDigest(m.SourceCompletionReceiptDigest) {
		return fail("manifest contains malformed scope, predicate, candidate-root or completion-receipt digest")
	}
	if m.CompletionDisposition != SourceDeclaredComplete {
		return fail("source has not declared complete enumeration")
	}
	if m.PageCount == 0 || uint64(len(pages)) != m.PageCount {
		return fail("enumeration page count is empty or differs from manifest")
	}

	allCandidates := make([]OwnerCandidate, 0)
	seenNextCursors := make(map[string]struct{}, len(pages))
	seenPageDigests := make(map[string]struct{}, len(pages))
	pageDigests := make([]string, len(pages))
	for i, page := range pages {
		if page.SourceRef != m.SourceRef || page.SnapshotRef != m.SnapshotRef ||
			page.RegistryRef != m.RegistryRef || page.RegistryEpoch != m.RegistryEpoch ||
			page.ProfileRef != m.ProfileRef || page.ScopeDigest != m.ScopeDigest ||
			page.PredicateDigest != m.PredicateDigest {
			return fail(fmt.Sprintf("page %d belongs to a different source, snapshot, epoch, profile, scope or predicate", i))
		}
		if page.PageIndex != uint64(i) {
			return fail(fmt.Sprintf("page index discontinuity at position %d", i))
		}
		if i == 0 {
			if page.RequestCursorDigest != "" {
				return fail("first page unexpectedly has a request cursor")
			}
		} else if page.RequestCursorDigest != pages[i-1].NextCursorDigest {
			return fail(fmt.Sprintf("cursor chain discontinuity before page %d", i))
		}
		isLast := i == len(pages)-1
		if page.Terminal != isLast {
			return fail(fmt.Sprintf("terminal marker is inconsistent at page %d", i))
		}
		if !isLast {
			if _, exists := seenNextCursors[page.NextCursorDigest]; exists {
				return fail(fmt.Sprintf("pagination cursor cycle detected at page %d", i))
			}
			seenNextCursors[page.NextCursorDigest] = struct{}{}
		}
		digest, err := EnumerationPageDigestV1(page)
		if err != nil {
			return fail(fmt.Sprintf("page %d is malformed: %v", i, err))
		}
		if digest != page.PageDigest {
			return fail(fmt.Sprintf("page %d digest mismatch", i))
		}
		if _, exists := seenPageDigests[digest]; exists {
			return fail(fmt.Sprintf("duplicate page digest at page %d", i))
		}
		seenPageDigests[digest] = struct{}{}
		pageDigests[i] = digest
		allCandidates = append(allCandidates, page.Candidates...)
	}
	if pageDigests[0] != m.FirstPageDigest || pageDigests[len(pageDigests)-1] != m.TerminalPageDigest {
		return fail("manifest first or terminal page digest mismatch")
	}
	root, err := CandidateRoot(allCandidates)
	if err != nil {
		return fail("cross-page candidate inventory is invalid: " + err.Error())
	}
	if root != m.CandidateRoot {
		return fail("aggregate candidate root differs from manifest")
	}

	var chain bytes.Buffer
	writeString(&chain, "MYCELIX-OWNER-ENUMERATION-CHAIN-V1")
	writeString(&chain, m.SourceRef)
	writeString(&chain, m.SnapshotRef)
	writeString(&chain, m.RegistryRef)
	writeString(&chain, m.RegistryEpoch)
	writeString(&chain, m.ProfileRef)
	writeString(&chain, m.ScopeDigest)
	writeString(&chain, m.PredicateDigest)
	for _, digest := range pageDigests { writeString(&chain, digest) }
	chainHash := sha256.Sum256(chain.Bytes())
	result.PageChainDigest = hex.EncodeToString(chainHash[:])
	result.Disposition = EnumerationChainWellFormed
	result.Reason = "enumeration page chain is structurally well formed; source authentication and completeness remain external"
	return finalizeEnumerationPageChainV1(result)
}

func finalizeEnumerationPageChainV1(result EnumerationPageChainResultV1) EnumerationPageChainResultV1 {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-OWNER-ENUMERATION-RESULT-V1")
	writeString(&b, string(result.Disposition))
	writeString(&b, result.Reason)
	for _, value := range []string{
		result.SourceRef, result.SnapshotRef, result.RegistryRef,
		result.RegistryEpoch, result.ProfileRef, result.ScopeDigest,
		result.PredicateDigest, result.CandidateRoot,
	} {
		writeString(&b, value)
	}
	_ = binary.Write(&b, binary.BigEndian, result.ExpectedPageCount)
	_ = binary.Write(&b, binary.BigEndian, result.ObservedPageCount)
	writeString(&b, result.SourceCompletionReceiptDigest)
	writeString(&b, string(result.CompletionDisposition))
	writeString(&b, result.ManifestDigest)
	writeString(&b, result.PageChainDigest)
	writeString(&b, result.PageChainAuthentication)
	writeString(&b, result.SourceCompletenessAuthentication)
	d := sha256.Sum256(b.Bytes())
	result.CertificateDigest = hex.EncodeToString(d[:])
	return result
}
