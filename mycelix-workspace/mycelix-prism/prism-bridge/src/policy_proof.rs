//! Proof-carrying semantic justification for emitted V2 ALLOW paths.
//!
//! This module is proof-only in the first tranche. It does not modify cBPF
//! emission or kernel installation. It establishes the next security contract:
//! every policy clause has exactly one explicit semantic owner.

use crate::seccomp::{SeccompSyscallPolicyV2, SeccompSyscallRuleV2};
use blake3::Hasher;
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum CapabilityClassV1 {
    Network = 1,
    Storage = 2,
    Clipboard = 3,
    Device = 4,
    Process = 5,
    Runtime = 6,
    Graphics = 7,
    Ipc = 8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum PolicySubsystemV1 {
    RendererIpc = 1,
    NetworkBroker = 2,
    StorageBroker = 3,
    Runtime = 4,
    Compositor = 5,
    BrowserCore = 6,
    SandboxBootstrap = 7,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum JustificationCodeV1 {
    RequiredRuntimeOperation = 1,
    ExplicitCapabilitySurface = 2,
    InterprocessTransport = 3,
    KernelAbiRequirement = 4,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct PolicyAtomV1 {
    pub syscall: i64,
    pub clause_digest: [u8; 32],
    pub capability: CapabilityClassV1,
    pub subsystem: PolicySubsystemV1,
    pub rationale: JustificationCodeV1,
}

impl PolicyAtomV1 {
    pub fn new(
        syscall: i64,
        clause_digest: [u8; 32],
        capability: CapabilityClassV1,
        subsystem: PolicySubsystemV1,
        rationale: JustificationCodeV1,
    ) -> Result<Self, PolicyProofError> {
        if syscall < 0 || syscall > i32::MAX as i64 || clause_digest == [0; 32] {
            return Err(PolicyProofError::InvalidAtom);
        }
        Ok(Self { syscall, clause_digest, capability, subsystem, rationale })
    }

    fn canonical_bytes(&self) -> Vec<u8> {
        fn field(out: &mut Vec<u8>, bytes: &[u8]) {
            out.extend_from_slice(&(bytes.len() as u64).to_be_bytes());
            out.extend_from_slice(bytes);
        }
        let mut out = Vec::new();
        field(&mut out, b"prism-policy-atom-v1");
        field(&mut out, &self.syscall.to_be_bytes());
        field(&mut out, &self.clause_digest);
        field(&mut out, &[self.capability as u8]);
        field(&mut out, &[self.subsystem as u8]);
        field(&mut out, &[self.rationale as u8]);
        out
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyJustificationSetV1 {
    pub policy_digest: [u8; 32],
    atoms: Vec<PolicyAtomV1>,
}

impl PolicyJustificationSetV1 {
    pub fn derive_clause_digest(rule: &SeccompSyscallRuleV2, clause_index: usize) -> Result<[u8; 32], PolicyProofError> {
        let clause = rule
            .clauses()
            .get(clause_index)
            .ok_or(PolicyProofError::ClauseOutOfRange)?;
        let mut hasher = Hasher::new();
        hasher.update(b"prism-v2-clause-identity-v1");
        hasher.update(&rule.syscall().to_be_bytes());
        hasher.update(&(clause.predicates().len() as u8).to_be_bytes());
        for predicate in clause.predicates() {
            hasher.update(&[predicate.arg_index()]);
            hasher.update(&predicate.mask().to_be_bytes());
            hasher.update(&predicate.value().to_be_bytes());
            hasher.update(&[predicate.op() as u8]);
        }
        Ok(*hasher.finalize().as_bytes())
    }

    pub fn new(
        policy: &SeccompSyscallPolicyV2,
        mut atoms: Vec<PolicyAtomV1>,
    ) -> Result<Self, PolicyProofError> {
        let policy_digest = policy.digest();
        atoms.sort_unstable();
        let mut seen = BTreeSet::new();
        for atom in &atoms {
            if !seen.insert((atom.syscall, atom.clause_digest)) {
                return Err(PolicyProofError::DuplicateAtom);
            }
        }

        let mut required = BTreeSet::new();
        for rule in policy.rules() {
            for index in 0..rule.clauses().len() {
                required.insert((rule.syscall(), Self::derive_clause_digest(rule, index)?));
            }
        }

        let supplied: BTreeSet<_> = atoms
            .iter()
            .map(|atom| (atom.syscall, atom.clause_digest))
            .collect();
        if supplied != required {
            if !supplied.is_subset(&required) {
                return Err(PolicyProofError::OrphanAtom);
            }
            return Err(PolicyProofError::MissingAtom);
        }

        Ok(Self { policy_digest, atoms })
    }

    pub fn policy_digest(&self) -> [u8; 32] { self.policy_digest }
    pub fn atoms(&self) -> &[PolicyAtomV1] { &self.atoms }

    pub fn digest(&self) -> [u8; 32] {
        let mut hasher = Hasher::new();
        hasher.update(b"prism-policy-justification-set-v1");
        hasher.update(&self.policy_digest);
        for atom in &self.atoms {
            hasher.update(&atom.canonical_bytes());
        }
        *hasher.finalize().as_bytes()
    }

    pub fn find(&self, syscall: i64, clause_digest: [u8; 32]) -> Option<&PolicyAtomV1> {
        self.atoms
            .iter()
            .find(|atom| atom.syscall == syscall && atom.clause_digest == clause_digest)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyProofError {
    InvalidAtom,
    ClauseOutOfRange,
    DuplicateAtom,
    MissingAtom,
    OrphanAtom,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seccomp::{
        SeccompArchitecture, SeccompArgPredicateV1, SeccompSyscallClauseV2,
        SeccompSyscallRuleV2,
    };

    fn policy() -> SeccompSyscallPolicyV2 {
        let predicate = SeccompArgPredicateV1::new(0, 0xff, 2).unwrap();
        let rule = SeccompSyscallRuleV2::new(libc::SYS_socket as i64, vec![predicate]).unwrap();
        SeccompSyscallPolicyV2::new(SeccompArchitecture::current().unwrap(), vec![rule]).unwrap()
    }

    fn atoms_for(policy: &SeccompSyscallPolicyV2) -> Vec<PolicyAtomV1> {
        policy
            .rules()
            .iter()
            .flat_map(|rule| {
                (0..rule.clauses().len()).map(move |i| {
                    let digest =
                        PolicyJustificationSetV1::derive_clause_digest(rule, i).unwrap();
                    PolicyAtomV1::new(
                        rule.syscall(),
                        digest,
                        CapabilityClassV1::Network,
                        PolicySubsystemV1::NetworkBroker,
                        JustificationCodeV1::ExplicitCapabilitySurface,
                    )
                    .unwrap()
                })
            })
            .collect()
    }

    #[test]
    fn every_policy_clause_requires_exactly_one_atom() {
        let policy = policy();
        let proof = PolicyJustificationSetV1::new(&policy, atoms_for(&policy)).unwrap();
        assert_eq!(proof.atoms().len(), 1);
        assert_eq!(proof.policy_digest(), policy.digest());
    }

    #[test]
    fn multi_clause_policy_requires_one_atom_per_clause() {
        let predicate_a = SeccompArgPredicateV1::new(0, 0xff, 2).unwrap();
        let predicate_b = SeccompArgPredicateV1::new(1, 0xff, 4).unwrap();
        let clauses = vec![
            SeccompSyscallClauseV2::new(vec![predicate_a]).unwrap(),
            SeccompSyscallClauseV2::new(vec![predicate_b]).unwrap(),
        ];
        let rule = SeccompSyscallRuleV2::new_with_clauses(
            libc::SYS_socket as i64,
            clauses,
        ).unwrap();
        let policy = SeccompSyscallPolicyV2::new(
            SeccompArchitecture::current().unwrap(),
            vec![rule],
        ).unwrap();

        let proof = PolicyJustificationSetV1::new(&policy, atoms_for(&policy)).unwrap();
        assert_eq!(proof.atoms().len(), 2);

        let one = proof.atoms().first().cloned().unwrap();
        let incomplete = PolicyJustificationSetV1::new(&policy, vec![one]);
        assert_eq!(incomplete, Err(PolicyProofError::MissingAtom));
    }

    #[test]
    fn missing_atom_fails_closed() {
        let policy = policy();
        assert_eq!(
            PolicyJustificationSetV1::new(&policy, Vec::new()),
            Err(PolicyProofError::MissingAtom)
        );
    }

    #[test]
    fn orphan_atom_fails_closed() {
        let policy = policy();
        let atom = PolicyAtomV1::new(
            libc::SYS_read as i64,
            [7; 32],
            CapabilityClassV1::Storage,
            PolicySubsystemV1::StorageBroker,
            JustificationCodeV1::ExplicitCapabilitySurface,
        ).unwrap();
        assert_eq!(
            PolicyJustificationSetV1::new(&policy, vec![atom]),
            Err(PolicyProofError::OrphanAtom)
        );
    }

    #[test]
    fn duplicate_atom_fails_closed() {
        let policy = policy();
        let atoms = atoms_for(&policy);
        assert_eq!(
            PolicyJustificationSetV1::new(&policy, vec![atoms[0].clone(), atoms[0].clone()]),
            Err(PolicyProofError::DuplicateAtom)
        );
    }

    #[test]
    fn rationale_mutation_changes_proof_identity() {
        let policy = policy();
        let mut atoms = atoms_for(&policy);
        let first = atoms[0].clone();
        let changed = PolicyAtomV1::new(
            first.syscall, first.clause_digest, first.capability,
            first.subsystem, JustificationCodeV1::KernelAbiRequirement
        ).unwrap();
        atoms[0] = changed;
        let a = PolicyJustificationSetV1::new(&policy, atoms_for(&policy)).unwrap();
        let b = PolicyJustificationSetV1::new(&policy, atoms).unwrap();
        assert_ne!(a.digest(), b.digest());
    }
}