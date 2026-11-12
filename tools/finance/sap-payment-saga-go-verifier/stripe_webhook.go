package sagaverifier

import (
	"crypto/hmac"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"strconv"
	"strings"
	"unicode/utf8"
)

const (
	StripeDefaultToleranceSecondsV1 uint64 = 300
	maxStripeSignatureHeaderBytesV1        = 16 * 1024
)

var (
	ErrStripeWebhookEmptySecretV1               = errors.New("Stripe webhook signing secret is empty")
	ErrStripeWebhookEmptyHeaderV1               = errors.New("Stripe-Signature header is empty")
	ErrStripeWebhookHeaderTooLargeV1            = errors.New("Stripe-Signature header exceeds limit")
	ErrStripeWebhookInvalidHeaderEncodingV1     = errors.New("Stripe-Signature header is not valid ASCII")
	ErrStripeWebhookInvalidHeaderFieldV1        = errors.New("Stripe-Signature header field is malformed")
	ErrStripeWebhookDuplicateTimestampV1        = errors.New("Stripe-Signature header has multiple timestamps")
	ErrStripeWebhookMissingTimestampV1          = errors.New("Stripe-Signature header has no timestamp")
	ErrStripeWebhookMissingV1SignatureV1        = errors.New("Stripe-Signature header has no v1 signature")
	ErrStripeWebhookInvalidTimestampV1          = errors.New("Stripe-Signature timestamp is invalid")
	ErrStripeWebhookInvalidSignatureEncodingV1  = errors.New("Stripe v1 signature is not 32-byte hexadecimal")
	ErrStripeWebhookInvalidToleranceV1          = errors.New("Stripe webhook timestamp tolerance must be positive")
	ErrStripeWebhookTimestampOutsideToleranceV1 = errors.New("Stripe webhook timestamp is outside tolerance")
	ErrStripeWebhookSignatureMismatchV1         = errors.New("Stripe webhook HMAC signature does not match")
)

type StripeWebhookVerificationV1 struct {
	TimestampUnixSeconds     uint64
	RawBodySHA256            string
	MatchingV1SignatureCount uint16
}

// VerifyStripeWebhookV1 verifies the HMAC over the exact raw HTTP body bytes.
// Call this before JSON decoding, normalization, or any payment mutation.
// Unknown header attributes (including legacy/future versions) are ignored;
// at least one v1 value must authenticate successfully.

// Verify against every currently active endpoint secret during secret rotation.
func VerifyStripeWebhookWithSecretsV1(
	rawBody []byte,
	signatureHeader string,
	activeSigningSecrets [][]byte,
	nowUnixSeconds uint64,
	toleranceSeconds uint64,
) (StripeWebhookVerificationV1, error) {
	nonEmpty := false
	for _, secret := range activeSigningSecrets {
		if len(secret) > 0 {
			nonEmpty = true
			break
		}
	}
	if !nonEmpty {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookEmptySecretV1
	}
	for _, secret := range activeSigningSecrets {
		if len(secret) == 0 {
			continue
		}
		verified, err := VerifyStripeWebhookV1(
			rawBody, signatureHeader, secret, nowUnixSeconds, toleranceSeconds,
		)
		if err == nil {
			return verified, nil
		}
		if err == ErrStripeWebhookSignatureMismatchV1 {
			continue
		}
		return StripeWebhookVerificationV1{}, err
	}
	return StripeWebhookVerificationV1{}, ErrStripeWebhookSignatureMismatchV1
}

func VerifyStripeWebhookV1(
	rawBody []byte,
	signatureHeader string,
	signingSecret []byte,
	nowUnixSeconds uint64,
	toleranceSeconds uint64,
) (StripeWebhookVerificationV1, error) {
	if len(signingSecret) == 0 {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookEmptySecretV1
	}
	if toleranceSeconds == 0 {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidToleranceV1
	}
	if signatureHeader == "" {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookEmptyHeaderV1
	}
	if len(signatureHeader) > maxStripeSignatureHeaderBytesV1 {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookHeaderTooLargeV1
	}
	if !utf8.ValidString(signatureHeader) {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidHeaderEncodingV1
	}
	for _, value := range signatureHeader {
		if value > 0x7f {
			return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidHeaderEncodingV1
		}
	}

	var timestampRaw string
	var timestamp uint64
	timestampSeen := false
	signatures := make([]string, 0, 2)
	for _, component := range strings.Split(signatureHeader, ",") {
		field := strings.TrimSpace(component)
		if field == "" {
			return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidHeaderFieldV1
		}
		name, value, ok := strings.Cut(field, "=")
		if !ok {
			return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidHeaderFieldV1
		}
		name = strings.TrimSpace(name)
		value = strings.TrimSpace(value)
		if name == "" || value == "" {
			return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidHeaderFieldV1
		}
		switch name {
		case "t":
			if timestampSeen {
				return StripeWebhookVerificationV1{}, ErrStripeWebhookDuplicateTimestampV1
			}
			parsed, err := strconv.ParseUint(value, 10, 64)
			if err != nil {
				return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidTimestampV1
			}
			timestampRaw, timestamp, timestampSeen = value, parsed, true
		case "v1":
			signatures = append(signatures, value)
		default:
			// Keep parsing forward-compatible with unknown signature versions/metadata.
		}
	}
	if !timestampSeen {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookMissingTimestampV1
	}
	if len(signatures) == 0 {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookMissingV1SignatureV1
	}

	preimage := make([]byte, 0, len(timestampRaw)+1+len(rawBody))
	preimage = append(preimage, timestampRaw...)
	preimage = append(preimage, '.')
	preimage = append(preimage, rawBody...)

	matching := uint16(0)
	anyWellFormed := false
	for _, candidate := range signatures {
		decoded, err := hex.DecodeString(candidate)
		if err != nil || len(decoded) != sha256.Size {
			continue
		}
		anyWellFormed = true
		mac := hmac.New(sha256.New, signingSecret)
		_, _ = mac.Write(preimage)
		expected := mac.Sum(nil)
		if hmac.Equal(expected, decoded) {
			if matching < ^uint16(0) {
				matching++
			}
		}
	}
	if !anyWellFormed {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookInvalidSignatureEncodingV1
	}
	if matching == 0 {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookSignatureMismatchV1
	}

	// Authenticate first, then apply the replay window to the signed timestamp.
	var delta uint64
	if nowUnixSeconds >= timestamp {
		delta = nowUnixSeconds - timestamp
	} else {
		delta = timestamp - nowUnixSeconds
	}
	if delta > toleranceSeconds {
		return StripeWebhookVerificationV1{}, ErrStripeWebhookTimestampOutsideToleranceV1
	}

	bodyDigest := sha256.Sum256(rawBody)
	return StripeWebhookVerificationV1{
		TimestampUnixSeconds:     timestamp,
		RawBodySHA256:            "sha256:" + hex.EncodeToString(bodyDigest[:]),
		MatchingV1SignatureCount: matching,
	}, nil
}
