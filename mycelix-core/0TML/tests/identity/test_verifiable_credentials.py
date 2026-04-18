# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
Unit tests for Verifiable Credentials System
Tests W3C VC issuance, verification, and management
"""

import pytest
from datetime import datetime, timezone, timedelta
from cryptography.hazmat.primitives.asymmetric import ed25519
from cryptography.hazmat.primitives import serialization

from zerotrustml.identity import (
    VCManager,
    VerifiableCredential,
    VCType,
)
from zerotrustml.identity.verifiable_credentials import (
    VCStatus,
    CredentialSubject,
)


class TestCredentialSubject:
    """Test CredentialSubject dataclass"""

    def test_create_subject(self):
        """Test creating a credential subject"""
        subject = CredentialSubject(
            id="did:mycelix:alice",
            claims={
                "name": "Alice",
                "verified": True
            }
        )

        assert subject.id == "did:mycelix:alice"
        assert subject.claims["name"] == "Alice"
        assert subject.claims["verified"] is True

    def test_subject_to_dict(self):
        """Test subject serialization"""
        subject = CredentialSubject(
            id="did:mycelix:bob",
            claims={
                "score": 42.5,
                "level": 3
            }
        )

        data = subject.to_dict()

        assert data["id"] == "did:mycelix:bob"
        assert data["score"] == 42.5
        assert data["level"] == 3

    def test_empty_claims(self):
        """Test subject with no claims"""
        subject = CredentialSubject(id="did:mycelix:test")

        assert subject.claims == {}
        data = subject.to_dict()
        assert data == {"id": "did:mycelix:test"}


class TestVerifiableCredential:
    """Test VerifiableCredential dataclass"""

    def test_create_credential(self):
        """Test creating a verifiable credential"""
        subject = CredentialSubject(
            id="did:mycelix:alice",
            claims={"verified": True}
        )

        vc = VerifiableCredential(
            id="urn:mycelix:credential:123",
            type=["VerifiableCredential", "VerifiedHuman"],
            issuer="did:mycelix:issuer",
            credential_subject=subject
        )

        assert vc.id == "urn:mycelix:credential:123"
        assert "VerifiedHuman" in vc.type
        assert vc.issuer == "did:mycelix:issuer"
        assert vc.status == VCStatus.ACTIVE

    def test_credential_expiration(self):
        """Test credential expiration checking"""
        subject = CredentialSubject(id="did:mycelix:alice")

        # Non-expired credential
        future = datetime.now(timezone.utc) + timedelta(days=30)
        vc_valid = VerifiableCredential(
            id="urn:test:1",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            expiration_date=future
        )

        assert not vc_valid.is_expired()

        # Expired credential
        past = datetime.now(timezone.utc) - timedelta(days=1)
        vc_expired = VerifiableCredential(
            id="urn:test:2",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            expiration_date=past
        )

        assert vc_expired.is_expired()

        # No expiration
        vc_permanent = VerifiableCredential(
            id="urn:test:3",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            expiration_date=None
        )

        assert not vc_permanent.is_expired()

    def test_credential_validity(self):
        """Test credential validity checking"""
        subject = CredentialSubject(id="did:mycelix:alice")

        # Valid credential (active, not expired, has proof)
        vc = VerifiableCredential(
            id="urn:test:1",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            status=VCStatus.ACTIVE,
            proof={"type": "Ed25519Signature2020"}
        )

        assert vc.is_valid()

        # Invalid: revoked
        vc_revoked = VerifiableCredential(
            id="urn:test:2",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            status=VCStatus.REVOKED,
            proof={"type": "Ed25519Signature2020"}
        )

        assert not vc_revoked.is_valid()

        # Invalid: no proof
        vc_no_proof = VerifiableCredential(
            id="urn:test:3",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            status=VCStatus.ACTIVE,
            proof=None
        )

        assert not vc_no_proof.is_valid()

    def test_credential_to_dict(self):
        """Test credential serialization"""
        subject = CredentialSubject(
            id="did:mycelix:alice",
            claims={"verified": True}
        )

        vc = VerifiableCredential(
            id="urn:test:123",
            type=["VerifiableCredential", "TestCredential"],
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            proof={"signatureValue": "abc123"}
        )

        data = vc.to_dict()

        assert "@context" in data
        assert data["id"] == "urn:test:123"
        assert "TestCredential" in data["type"]
        assert data["issuer"] == "did:mycelix:issuer"
        assert data["credentialSubject"]["id"] == "did:mycelix:alice"
        assert "proof" in data
        assert "mycelix" in data
        assert data["mycelix"]["status"] == "Active"

    def test_canonical_json(self):
        """Test canonical JSON generation for signing"""
        subject = CredentialSubject(id="did:mycelix:alice")

        vc = VerifiableCredential(
            id="urn:test:123",
            issuer="did:mycelix:issuer",
            credential_subject=subject
        )

        canonical = vc.get_canonical_json()

        # Should be valid JSON
        import json
        parsed = json.loads(canonical)
        assert parsed["id"] == "urn:test:123"

        # Should not include proof
        assert "proof" not in canonical


class TestVCManager:
    """Test VCManager functionality"""

    def setup_method(self):
        """Set up test fixtures"""
        # Generate issuer keys
        self.issuer_private = ed25519.Ed25519PrivateKey.generate()
        self.issuer_public = self.issuer_private.public_key()

        self.issuer_private_bytes = self.issuer_private.private_bytes(
            encoding=serialization.Encoding.Raw,
            format=serialization.PrivateFormat.Raw,
            encryption_algorithm=serialization.NoEncryption()
        )

        self.issuer_public_bytes = self.issuer_public.public_bytes(
            encoding=serialization.Encoding.Raw,
            format=serialization.PublicFormat.Raw
        )

        self.issuer_did = "did:mycelix:issuer"
        self.subject_did = "did:mycelix:alice"

        self.manager = VCManager()

    def test_issue_credential(self):
        """Test issuing a verifiable credential"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.VERIFIED_HUMAN,
            claims={"verified": True, "method": "VerifiedHumanity.org"},
            validity_days=365
        )

        assert vc is not None
        assert vc.issuer == self.issuer_did
        assert vc.credential_subject.id == self.subject_did
        assert "VerifiedHuman" in vc.type
        assert vc.proof is not None
        assert vc.proof["type"] == "Ed25519Signature2020"
        assert "signatureValue" in vc.proof

    def test_verify_valid_credential(self):
        """Test verifying a valid credential"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.REPUTATION_SCORE,
            claims={"score": 75}
        )

        # Verify with correct public key
        is_valid = self.manager.verify_credential(vc, self.issuer_public_bytes)
        assert is_valid

    def test_verify_invalid_signature(self):
        """Test verification fails with wrong public key"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.ATTESTATION,
            claims={"attested": True}
        )

        # Generate different key
        wrong_key = ed25519.Ed25519PrivateKey.generate()
        wrong_public = wrong_key.public_key().public_bytes(
            encoding=serialization.Encoding.Raw,
            format=serialization.PublicFormat.Raw
        )

        # Verification should fail
        is_valid = self.manager.verify_credential(vc, wrong_public)
        assert not is_valid

    def test_verify_tampered_credential(self):
        """Test verification fails with tampered data"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.MEMBERSHIP,
            claims={"member": True}
        )

        # Tamper with claims
        vc.credential_subject.claims["member"] = False

        # Verification should fail
        is_valid = self.manager.verify_credential(vc, self.issuer_public_bytes)
        assert not is_valid

    def test_credential_without_proof(self):
        """Test verification fails for credential without proof"""
        subject = CredentialSubject(id=self.subject_did)

        vc = VerifiableCredential(
            id="urn:test:123",
            issuer=self.issuer_did,
            credential_subject=subject,
            proof=None  # No proof
        )

        is_valid = self.manager.verify_credential(vc, self.issuer_public_bytes)
        assert not is_valid

    def test_revoke_credential(self):
        """Test revoking a credential"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.ACHIEVEMENT,
            claims={"achievement": "Test Completed"}
        )

        # Revoke
        success = self.manager.revoke_credential(
            vc.id,
            reason="Test revocation"
        )

        assert success

        # Check status
        revoked_vc = self.manager.get_credential(vc.id)
        assert revoked_vc.status == VCStatus.REVOKED
        assert not revoked_vc.is_valid()

    def test_revoke_nonexistent_credential(self):
        """Test revoking non-existent credential"""
        success = self.manager.revoke_credential(
            "urn:nonexistent:123",
            reason="Test"
        )

        assert not success

    def test_get_credential(self):
        """Test retrieving a credential by ID"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.EDUCATION,
            claims={"degree": "PhD", "institution": "Test University"}
        )

        # Retrieve
        retrieved = self.manager.get_credential(vc.id)

        assert retrieved is not None
        assert retrieved.id == vc.id
        assert retrieved.issuer == vc.issuer
        assert retrieved.credential_subject.id == vc.credential_subject.id

    def test_get_nonexistent_credential(self):
        """Test retrieving non-existent credential"""
        retrieved = self.manager.get_credential("urn:nonexistent:123")
        assert retrieved is None

    def test_list_credentials_for_subject(self):
        """Test listing all credentials for a subject"""
        # Issue multiple credentials
        vc1 = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.VERIFIED_HUMAN,
            claims={"verified": True}
        )

        vc2 = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.GITCOIN_PASSPORT,
            claims={"score": 42.5}
        )

        # Issue credential for different subject
        vc3 = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did="did:mycelix:bob",
            vc_type=VCType.MEMBERSHIP,
            claims={"member": True}
        )

        # List credentials for alice
        alice_credentials = self.manager.list_credentials_for_subject(self.subject_did)

        assert len(alice_credentials) == 2
        assert vc1.id in [c.id for c in alice_credentials]
        assert vc2.id in [c.id for c in alice_credentials]
        assert vc3.id not in [c.id for c in alice_credentials]

    def test_different_credential_types(self):
        """Test issuing different types of credentials"""
        credential_types = [
            (VCType.VERIFIED_HUMAN, {"verified": True}),
            (VCType.GITCOIN_PASSPORT, {"score": 42.5}),
            (VCType.REPUTATION_SCORE, {"reputation": 85}),
            (VCType.GOVERNANCE_ELIGIBILITY, {"eligible": True}),
            (VCType.ATTESTATION, {"attested": True}),
            (VCType.MEMBERSHIP, {"member": True}),
            (VCType.ACHIEVEMENT, {"achievement": "Test"}),
            (VCType.EDUCATION, {"degree": "PhD"}),
            (VCType.EMPLOYMENT, {"employer": "Mycelix"}),
        ]

        for vc_type, claims in credential_types:
            vc = self.manager.issue_credential(
                issuer_did=self.issuer_did,
                issuer_private_key=self.issuer_private_bytes,
                subject_did=self.subject_did,
                vc_type=vc_type,
                claims=claims
            )

            assert vc_type.value in vc.type
            assert self.manager.verify_credential(vc, self.issuer_public_bytes)

    def test_credential_with_validity_period(self):
        """Test credential with expiration"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.MEMBERSHIP,
            claims={"member": True},
            validity_days=30
        )

        assert vc.expiration_date is not None
        assert not vc.is_expired()

        # Check expiration is approximately 30 days from now
        expected_expiration = datetime.now(timezone.utc) + timedelta(days=30)
        time_diff = abs((vc.expiration_date - expected_expiration).total_seconds())
        assert time_diff < 10  # Within 10 seconds

    def test_credential_without_expiration(self):
        """Test credential with no expiration"""
        vc = self.manager.issue_credential(
            issuer_did=self.issuer_did,
            issuer_private_key=self.issuer_private_bytes,
            subject_did=self.subject_did,
            vc_type=VCType.ATTESTATION,
            claims={"permanent": True},
            validity_days=None
        )

        assert vc.expiration_date is None
        assert not vc.is_expired()

    def test_multiple_subjects(self):
        """Test managing credentials for multiple subjects"""
        subjects = [
            "did:mycelix:alice",
            "did:mycelix:bob",
            "did:mycelix:carol"
        ]

        # Issue credentials for each subject
        for subject in subjects:
            self.manager.issue_credential(
                issuer_did=self.issuer_did,
                issuer_private_key=self.issuer_private_bytes,
                subject_did=subject,
                vc_type=VCType.MEMBERSHIP,
                claims={"member": True}
            )

        # Verify each subject has exactly one credential
        for subject in subjects:
            credentials = self.manager.list_credentials_for_subject(subject)
            assert len(credentials) == 1
            assert credentials[0].credential_subject.id == subject


class TestVCStatusTransitions:
    """Test credential status transitions"""

    def setup_method(self):
        """Set up test fixtures"""
        self.issuer_private = ed25519.Ed25519PrivateKey.generate()
        self.issuer_private_bytes = self.issuer_private.private_bytes(
            encoding=serialization.Encoding.Raw,
            format=serialization.PrivateFormat.Raw,
            encryption_algorithm=serialization.NoEncryption()
        )

        self.manager = VCManager()

    def test_active_to_revoked(self):
        """Test transitioning from active to revoked"""
        vc = self.manager.issue_credential(
            issuer_did="did:mycelix:issuer",
            issuer_private_key=self.issuer_private_bytes,
            subject_did="did:mycelix:subject",
            vc_type=VCType.ATTESTATION,
            claims={"test": True}
        )

        assert vc.status == VCStatus.ACTIVE

        self.manager.revoke_credential(vc.id, "Test revocation")

        revoked = self.manager.get_credential(vc.id)
        assert revoked.status == VCStatus.REVOKED

    def test_expired_credential_validity(self):
        """Test expired credential is not valid"""
        subject = CredentialSubject(id="did:mycelix:alice")

        past = datetime.now(timezone.utc) - timedelta(days=1)
        vc = VerifiableCredential(
            id="urn:test:123",
            issuer="did:mycelix:issuer",
            credential_subject=subject,
            status=VCStatus.ACTIVE,
            expiration_date=past,
            proof={"type": "Ed25519Signature2020"}
        )

        # Even though status is ACTIVE and proof exists,
        # credential is invalid due to expiration
        assert not vc.is_valid()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
