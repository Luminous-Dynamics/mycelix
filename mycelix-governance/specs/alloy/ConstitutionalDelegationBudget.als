module ConstitutionalDelegationBudget

/*
Structural model for delegation attenuation and shared lineage use budgets.
The model represents use allowances as finite slot sets rather than integer
counters, making duplication/overlap directly visible to Alloy.
*/

open util/ordering[Scope] as ScopeOrd
open util/ordering[Time] as TimeOrd
open util/ordering[Depth] as DepthOrd

abstract sig Power {}
sig DelegablePower, NonDelegablePower extends Power {}

sig HolderClass {
  allowed: set Power
}

one sig AutomatedClass extends HolderClass {}

sig CapId {}
sig Scope {}
sig Time {}
sig Depth {}

sig Capability {
  id: one CapId,
  holderClass: one HolderClass,
  power: one Power,
  scope: one Scope,
  expiry: one Time,
  depth: one Depth,
  parent: lone Capability
}

fact UniqueCapabilityId {
  all disj a, b: Capability | a.id != b.id
}

fact DelegationIsAcyclic {
  no c: Capability | c in c.^parent
}

fact AutomatedClassHasNoConstitutionalPower {
  no AutomatedClass.allowed
}

fact HolderClassMustAllowPower {
  all c: Capability | c.power in c.holderClass.allowed
}

pred NarrowerOrSame[child, parent: Scope] {
  child = parent or child in parent.^ScopeOrd/next
}

pred NoLaterThan[child, parent: Time] {
  child = parent or parent in child.^TimeOrd/next
}

pred StrictlyLessDepth[child, parent: Depth] {
  parent in child.^DepthOrd/next
}

fact DelegationAttenuates {
  all c: Capability | some c.parent implies {
    c.power = c.parent.power
    NarrowerOrSame[c.scope, c.parent.scope]
    NoLaterThan[c.expiry, c.parent.expiry]
    StrictlyLessDepth[c.depth, c.parent.depth]
    c.power in DelegablePower
  }
}

fact NondelegablePowersHaveNoDelegatedChildren {
  no c: Capability | some c.parent and c.power in NonDelegablePower
}

/*
A root budget is bound to one root authorization and one finality domain.
Budget identity is canonical: two roots cannot independently instantiate the
same budget ID and thereby mint duplicate allowances.
*/

sig BudgetId {}
sig RootAuthorization {}
sig FinalityDomain {}
sig UseSlot {}

sig RootBudget {
  id: one BudgetId,
  authorization: one RootAuthorization,
  finalityDomain: one FinalityDomain,
  slots: some UseSlot
}

fact CanonicalRootBudgetIdentity {
  all disj a, b: RootBudget | {
    a.id != b.id
    a.authorization != b.authorization
  }
}

/*
An Allocation is a finalized subdivision of a root budget. A child allocation
must stay inside its parent allocation. Top-level allocations have no parent.
Sibling allocations are disjoint; descendants may overlap ancestors only by
being subsets of them.
*/

sig Allocation {
  root: one RootBudget,
  parent: lone Allocation,
  slots: some UseSlot
}

fact AllocationTreeIsAcyclic {
  no a: Allocation | a in a.^parent
}

fact AllocationAttenuates {
  all a: Allocation | {
    a.slots in a.root.slots
    some a.parent implies {
      a.root = a.parent.root
      a.slots in a.parent.slots
    }
  }
}

fact SiblingAllocationsAreDisjoint {
  all disj a, b: Allocation |
    (a.root = b.root and a.parent = b.parent) implies no (a.slots & b.slots)
}

/*
Finalized uses consume exactly one slot. A root-level use may consume only a
slot not already delegated to a top-level child allocation.
*/

sig FinalizedUse {
  root: one RootBudget,
  allocation: lone Allocation,
  slot: one UseSlot
}

fact FinalizedUseMustBeAuthorizedByItsLedger {
  all u: FinalizedUse | {
    some u.allocation implies {
      u.root = u.allocation.root
      u.slot in u.allocation.slots
    }
    no u.allocation implies {
      u.slot in u.root.slots
      no a: Allocation |
        a.root = u.root and no a.parent and u.slot in a.slots
    }
  }
}

fact AUseSlotFinalizesAtMostOncePerRoot {
  all disj a, b: FinalizedUse |
    a.root = b.root implies a.slot != b.slot
}

assert DelegationNeverChangesPower {
  all c: Capability | some c.parent implies c.power = c.parent.power
}

assert DelegationNeverBroadensScope {
  all c: Capability | some c.parent implies
    NarrowerOrSame[c.scope, c.parent.scope]
}

assert DelegationNeverOutlivesParent {
  all c: Capability | some c.parent implies
    NoLaterThan[c.expiry, c.parent.expiry]
}

assert DelegationDepthStrictlyDecreases {
  all c: Capability | some c.parent implies
    StrictlyLessDepth[c.depth, c.parent.depth]
}

assert NondelegablePowerCannotAppearBelowRoot {
  no c: Capability | some c.parent and c.power in NonDelegablePower
}

assert AutomatedClassCannotReceiveCapability {
  no c: Capability | c.holderClass = AutomatedClass
}

assert EveryAllocationStaysInsideRootBudget {
  all a: Allocation | a.slots in a.root.slots
}

assert RedelegationCannotInventSlots {
  all a: Allocation | some a.parent implies a.slots in a.parent.slots
}

assert SiblingsCannotDuplicateAllowance {
  no disj a, b: Allocation |
    a.root = b.root and a.parent = b.parent and some (a.slots & b.slots)
}

assert RootAllowanceCannotFinalizeTwice {
  no disj a, b: FinalizedUse |
    a.root = b.root and a.slot = b.slot
}

assert EveryFinalizedUseConsumesARootSlot {
  all u: FinalizedUse | u.slot in u.root.slots
}

/*
Expected-SAT witness for a nontrivial delegation + budget universe. This guards
against interpreting vacuous checks over an empty structural core as evidence.
*/
pred NontrivialDelegationAndBudget {
  some c: Capability | some c.parent
  some r: RootBudget | {
    some disj a, b: Allocation | {
      a.root = r
      b.root = r
      no a.parent
      no b.parent
    }
    some u: FinalizedUse | u.root = r
  }
}

/*
Negative control: matching names alone do NOT imply shared accounting. These
weak ledgers intentionally have no canonical-root constraint. Alloy should be
able to find two ledgers with the same ID and overlapping locally-visible slots.
*/

sig WeakLedger {
  id: one BudgetId,
  locallyAvailable: set UseSlot
}

pred SameBudgetNameCanStillDuplicateLocalAllowance {
  some disj a, b: WeakLedger |
    a.id = b.id and some (a.locallyAvailable & b.locallyAvailable)
}

run NontrivialDelegationAndBudget for 8 Capability, 8 CapId, 5 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth,
  3 RootBudget, 8 Allocation, 8 FinalizedUse, 10 UseSlot, 3 BudgetId,
  3 RootAuthorization, 3 FinalityDomain expect 1

run SameBudgetNameCanStillDuplicateLocalAllowance for 5 WeakLedger, 4 BudgetId, 5 UseSlot expect 1

check DelegationNeverChangesPower for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0
check DelegationNeverBroadensScope for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0
check DelegationNeverOutlivesParent for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0
check DelegationDepthStrictlyDecreases for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0
check NondelegablePowerCannotAppearBelowRoot for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0
check AutomatedClassCannotReceiveCapability for 10 Capability, 10 CapId, 6 HolderClass, 6 Power,
  exactly 5 Scope, exactly 5 Time, exactly 5 Depth expect 0

check EveryAllocationStaysInsideRootBudget for 5 RootBudget, 12 Allocation, 12 UseSlot,
  5 BudgetId, 5 RootAuthorization, 5 FinalityDomain expect 0
check RedelegationCannotInventSlots for 5 RootBudget, 12 Allocation, 12 UseSlot,
  5 BudgetId, 5 RootAuthorization, 5 FinalityDomain expect 0
check SiblingsCannotDuplicateAllowance for 5 RootBudget, 12 Allocation, 12 UseSlot,
  5 BudgetId, 5 RootAuthorization, 5 FinalityDomain expect 0
check RootAllowanceCannotFinalizeTwice for 5 RootBudget, 12 Allocation, 12 FinalizedUse,
  12 UseSlot, 5 BudgetId, 5 RootAuthorization, 5 FinalityDomain expect 0
check EveryFinalizedUseConsumesARootSlot for 5 RootBudget, 12 Allocation, 12 FinalizedUse,
  12 UseSlot, 5 BudgetId, 5 RootAuthorization, 5 FinalityDomain expect 0
