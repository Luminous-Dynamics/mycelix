# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
"""
Verifiable Credentials (VC) System for Mycelix
W3C Verifiable Credentials Data Model Implementation

Phase 1: Basic VCs for identity attestations
Phase 2+: ZK proofs, selective disclosure, revocation
"""

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from enum import Enum
from typing import Dict, List, Optional, Any
from cryptography.hazmat.primitives.asymmetric import ed25519
from cryptography.hazmat.primitives import serialization


class VCType(Enum):
    """Types of Verifiable Credentials"""
    VERIFIED_HUMAN = "VerifiedHuman"  # VerifiedHumanity.org
    GITCOIN_PASSPORT = "GitcoinPassport"
    REPUTATION_SCORE = "ReputationScore"
    GOVERNANCE_ELIGIBILITY = "GovernanceEligibility"
    ATTESTATION = "Attestation"
    MEMBERSHIP = "Membership"
    ACHIEVEMENT = "Achievement"
    EDUCATION = "Education"
    EMPLOYMENT = "Employment"


class VCStatus(Enum):
    """Status of a Verifiable Credential"""
    ACTIVE = "Active"
    SUSPENDED = "Suspended"
    REVOKED = "Revoked"
    EXPIRED = "Expired"


@dataclass
class CredentialSubject:
    """
    The subject of a Verifiable Credential

    Contains claims about the DID holder
    """
    id: str  # DID of the subject
    claims: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        """Serialize subject"""
        return {
            "id": self.id,
            **self.claims
        }


@dataclass
class VerifiableCredential:
    """
    W3C Verifiable Credential

    A cryptographically signed attestation about a DID holder
    """
    # W3C Required Fields
    context: List[str] = field(default_factory=lambda: [
        "https://www.w3.org/2018/credentials/v1",
        "https://mycelix.net/credentials/v1"
    ])
    id: str = ""  # Unique credential ID
    type: List[str] = field(default_factory=lambda: ["VerifiableCredential"])
    issuer: str = ""  # DID of issuer
    issuance_date: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    credential_subject: Optional[CredentialSubject] = None

    # Optional Fields
    expiration_date: Optional[datetime] = None
    status: VCStatus = VCStatus.ACTIVE

    # Proof (cryptographic signature)
    proof: Optional[Dict] = None

    # Mycelix Extensions
    mycelix_metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self, include_proof: bool = True) -> Dict:
        """
        Serialize credential to W3C format

        Args:
            include_proof: Whether to include the proof

        Returns:
            W3C-compliant credential dictionary
        """
        doc = {
            "@context": self.context,
            "id": self.id,
            "type": self.type,
            "issuer": self.issuer,
            "issuanceDate": self.issuance_date.isoformat(),
            "credentialSubject": self.credential_subject.to_dict() if self.credential_subject else {}
        }

        if self.expiration_date:
            doc["expirationDate"] = self.expiration_date.isoformat()

        if include_proof and self.proof:
            doc["proof"] = self.proof

        # Mycelix extensions
        doc["mycelix"] = {
            "status": self.status.value,
            **self.mycelix_metadata
        }

        return doc

    def is_expired(self) -> bool:
        """Check if credential has expired"""
        if not self.expiration_date:
            return False
        return datetime.now(timezone.utc) > self.expiration_date

    def is_valid(self) -> bool:
        """Check if credential is currently valid"""
        return (
            self.status == VCStatus.ACTIVE and
            not self.is_expired() and
            self.proof is not None
        )

    def get_canonical_json(self) -> str:
        """Get canonical JSON for signing"""
        doc = self.to_dict(include_proof=False)
        return json.dumps(doc, sort_keys=True, separators=(',', ':'))


class VCManager:
    """
    Manages Verifiable Credentials

    Issues, verifies, and stores credentials
    """

    def __init__(self, storage_backend=None):
        """
        Initialize VC Manager

        Args:
            storage_backend: Backend for persistent storage
        """
        self.storage = storage_backend or {}  # In-memory for Phase 1

    def issue_credential(
        self,
        issuer_did: str,
        issuer_private_key: bytes,
        subject_did: str,
        vc_type: VCType,
        claims: Dict[str, Any],
        validity_days: Optional[int] = None
    ) -> VerifiableCredential:
        """
        Issue a new Verifiable Credential

        Args:
            issuer_did: DID of the issuer
            issuer_private_key: Private key for signing
            subject_did: DID of the subject
            vc_type: Type of credential
            claims: Claims about the subject
            validity_days: Days until expiration (None = no expiration)

        Returns:
            Signed VerifiableCredential
        """
        # Create credential subject
        subject = CredentialSubject(
            id=subject_did,
            claims=claims
        )

        # Generate credential ID
        credential_id = self._generate_credential_id(issuer_did, subject_did, vc_type)

        # Calculate expiration
        expiration = None
        if validity_days:
            expiration = datetime.now(timezone.utc) + timedelta(days=validity_days)

        # Create credential
        vc = VerifiableCredential(
            id=credential_id,
            type=["VerifiableCredential", vc_type.value],
            issuer=issuer_did,
            credential_subject=subject,
            expiration_date=expiration
        )

        # Sign credential
        self._sign_credential(vc, issuer_private_key)

        # Store credential
        self.storage[credential_id] = vc.to_dict()

        return vc

    def verify_credential(
        self,
        vc: VerifiableCredential,
        issuer_public_key: bytes
    ) -> bool:
        """
        Verify a Verifiable Credential's signature

        Args:
            vc: Credential to verify
            issuer_public_key: Issuer's public key

        Returns:
            True if signature is valid
        """
        if not vc.proof:
            return False

        if not vc.is_valid():
            return False

        try:
            # Get canonical JSON (without proof)
            canonical_json = vc.get_canonical_json()
            message = canonical_json.encode('utf-8')

            # Extract signature from proof
            signature_hex = vc.proof.get('signatureValue')
            if not signature_hex:
                return False

            signature = bytes.fromhex(signature_hex)

            # Verify signature
            public_key_obj = ed25519.Ed25519PublicKey.from_public_bytes(issuer_public_key)
            public_key_obj.verify(signature, message)

            return True
        except Exception:
            return False

    def revoke_credential(self, credential_id: str, reason: str = "") -> bool:
        """
        Revoke a credential

        Args:
            credential_id: ID of credential to revoke
            reason: Reason for revocation

        Returns:
            True if revocation successful
        """
        if credential_id not in self.storage:
            return False

        vc_dict = self.storage[credential_id]
        vc_dict["mycelix"]["status"] = VCStatus.REVOKED.value
        vc_dict["mycelix"]["revocation_reason"] = reason
        vc_dict["mycelix"]["revoked_at"] = datetime.now(timezone.utc).isoformat()

        self.storage[credential_id] = vc_dict

        return True

    def get_credential(self, credential_id: str) -> Optional[VerifiableCredential]:
        """
        Retrieve a credential by ID

        Args:
            credential_id: Credential ID

        Returns:
            VerifiableCredential or None if not found
        """
        vc_dict = self.storage.get(credential_id)
        if not vc_dict:
            return None

        return self._dict_to_vc(vc_dict)

    def list_credentials_for_subject(self, subject_did: str) -> List[VerifiableCredential]:
        """
        List all credentials for a subject DID

        Args:
            subject_did: Subject's DID

        Returns:
            List of VerifiableCredentials
        """
        credentials = []
        for vc_dict in self.storage.values():
            if vc_dict.get("credentialSubject", {}).get("id") == subject_did:
                vc = self._dict_to_vc(vc_dict)
                if vc:
                    credentials.append(vc)

        return credentials

    def _sign_credential(self, vc: VerifiableCredential, private_key: bytes):
        """
        Sign a credential with issuer's private key

        Args:
            vc: Credential to sign
            private_key: Issuer's private key
        """
        # Get canonical JSON for signing
        canonical_json = vc.get_canonical_json()
        message = canonical_json.encode('utf-8')

        # Sign with Ed25519
        private_key_obj = ed25519.Ed25519PrivateKey.from_private_bytes(private_key)
        signature = private_key_obj.sign(message)

        # Create proof
        vc.proof = {
            "type": "Ed25519Signature2020",
            "created": datetime.now(timezone.utc).isoformat(),
            "verificationMethod": f"{vc.issuer}#keys-1",
            "proofPurpose": "assertionMethod",
            "signatureValue": signature.hex()
        }

    @staticmethod
    def _generate_credential_id(
        issuer_did: str,
        subject_did: str,
        vc_type: VCType
    ) -> str:
        """Generate a unique credential ID"""
        data = f"{issuer_did}:{subject_did}:{vc_type.value}:{datetime.now(timezone.utc).isoformat()}"
        hash_bytes = hashlib.sha256(data.encode('utf-8')).digest()
        return f"urn:mycelix:credential:{hash_bytes.hex()[:16]}"

    @staticmethod
    def _dict_to_vc(vc_dict: Dict) -> Optional[VerifiableCredential]:
        """Convert dictionary to VerifiableCredential object"""
        try:
            subject_data = vc_dict.get("credentialSubject", {})
            subject_id = subject_data.get("id")
            subject_claims = {k: v for k, v in subject_data.items() if k != "id"}

            subject = CredentialSubject(
                id=subject_id,
                claims=subject_claims
            )

            issuance_date = datetime.fromisoformat(vc_dict["issuanceDate"].replace('Z', '+00:00'))
            expiration_date = None
            if "expirationDate" in vc_dict:
                expiration_date = datetime.fromisoformat(vc_dict["expirationDate"].replace('Z', '+00:00'))

            mycelix_data = vc_dict.get("mycelix", {})
            status = VCStatus(mycelix_data.get("status", "Active"))

            vc = VerifiableCredential(
                context=vc_dict.get("@context", []),
                id=vc_dict["id"],
                type=vc_dict["type"],
                issuer=vc_dict["issuer"],
                issuance_date=issuance_date,
                credential_subject=subject,
                expiration_date=expiration_date,
                status=status,
                proof=vc_dict.get("proof"),
                mycelix_metadata={k: v for k, v in mycelix_data.items() if k != "status"}
            )

            return vc
        except Exception:
            return None


# Example usage
if __name__ == "__main__":
    from cryptography.hazmat.primitives.asymmetric import ed25519

    print("=== Verifiable Credentials Example ===\n")

    # Create issuer keys
    issuer_private = ed25519.Ed25519PrivateKey.generate()
    issuer_public = issuer_private.public_key()

    issuer_private_bytes = issuer_private.private_bytes(
        encoding=serialization.Encoding.Raw,
        format=serialization.PrivateFormat.Raw,
        encryption_algorithm=serialization.NoEncryption()
    )

    issuer_public_bytes = issuer_public.public_bytes(
        encoding=serialization.Encoding.Raw,
        format=serialization.PublicFormat.Raw
    )

    # DIDs
    issuer_did = "did:mycelix:issuer123"
    subject_did = "did:mycelix:alice456"

    # Create VC Manager
    manager = VCManager()

    # Issue a VerifiedHuman credential
    print("Issuing VerifiedHuman credential...")
    vc = manager.issue_credential(
        issuer_did=issuer_did,
        issuer_private_key=issuer_private_bytes,
        subject_did=subject_did,
        vc_type=VCType.VERIFIED_HUMAN,
        claims={
            "verifiedHuman": True,
            "verificationMethod": "VerifiedHumanity.org",
            "verificationDate": datetime.now(timezone.utc).isoformat()
        },
        validity_days=365
    )

    print(f"Credential ID: {vc.id}")
    print(f"Type: {vc.type}")
    print(f"Issuer: {vc.issuer}")
    print(f"Subject: {vc.credential_subject.id}")
    print(f"Valid: {vc.is_valid()}")

    # Verify credential
    print("\nVerifying credential signature...")
    is_valid = manager.verify_credential(vc, issuer_public_bytes)
    print(f"Signature valid: {is_valid}")

    # Export to JSON
    print("\nCredential as JSON:")
    vc_json = json.dumps(vc.to_dict(), indent=2)
    print(vc_json[:500] + "...")

    # Issue a Gitcoin Passport credential
    print("\n\nIssuing Gitcoin Passport credential...")
    vc2 = manager.issue_credential(
        issuer_did=issuer_did,
        issuer_private_key=issuer_private_bytes,
        subject_did=subject_did,
        vc_type=VCType.GITCOIN_PASSPORT,
        claims={
            "passportScore": 42.5,
            "stamps": ["Google", "Twitter", "GitHub", "Discord"],
            "address": "0x1234567890123456789012345678901234567890"
        },
        validity_days=90
    )

    print(f"Credential ID: {vc2.id}")
    print(f"Claims: {vc2.credential_subject.claims}")

    # List all credentials for subject
    print("\n\nAll credentials for subject:")
    all_credentials = manager.list_credentials_for_subject(subject_did)
    for credential in all_credentials:
        print(f"  - {credential.type[1]} (ID: {credential.id[:40]}...)")

    # Revoke a credential
    print("\n\nRevoking VerifiedHuman credential...")
    manager.revoke_credential(vc.id, reason="Test revocation")

    # Check validity after revocation
    updated_vc = manager.get_credential(vc.id)
    print(f"Valid after revocation: {updated_vc.is_valid()}")
    print(f"Status: {updated_vc.status.value}")
