import assert from "node:assert/strict";
import test from "node:test";
import { decode } from "@msgpack/msgpack";
import { decodeZomePayload, encodeZomeResponse } from "../src/wire";

function hash(seed: number): Uint8Array {
  return Uint8Array.from({ length: 39 }, (_, index) => (seed + index) & 0xff);
}

test("ActionHash payload remains MessagePack binary", () => {
  const actionHash = hash(11);
  const encoded = encodeZomeResponse(actionHash);
  const decoded = decode(encoded);

  assert.ok(decoded instanceof Uint8Array);
  assert.deepEqual(Array.from(decoded), Array.from(actionHash));
  assert.deepEqual(
    Array.from(decodeZomePayload(encoded) as Uint8Array),
    Array.from(actionHash),
  );
});

test("listing output keeps binary hash fields", () => {
  const output = {
    listing_hash: hash(21),
    seller_agent_id: hash(31),
    listing: {
      title: "Test listing",
      description: "Wire fixture",
      price_cents: 1999,
      category: "Electronics",
      photos_ipfs_cids: ["bafyfixture"],
      quantity_available: 2,
      status: "active",
      epistemic: {
        empirical: "E1Testimonial",
        normative: "N0Personal",
        materiality: "M1Temporal",
      },
      created_at: 1,
      updated_at: 1,
    },
  };

  const decoded = decodeZomePayload(encodeZomeResponse(output)) as typeof output;
  assert.ok(decoded.listing_hash instanceof Uint8Array);
  assert.ok(decoded.seller_agent_id instanceof Uint8Array);
  assert.equal(decoded.listing.status, "active");
});

test("transaction status uses the integrity zome lowercase representation", () => {
  const output = {
    transaction_hash: hash(41),
    transaction: {
      buyer: hash(51),
      seller: hash(61),
      listing_hash: hash(71),
      quantity: 1,
      total_price_cents: 1999,
      status: "pending",
      created_at: 1,
      updated_at: 1,
      tracking_info: null,
      epistemic: {
        empirical: "E1Testimonial",
        normative: "N1Communal",
        materiality: "M1Temporal",
      },
    },
  };

  const decoded = decodeZomePayload(encodeZomeResponse(output)) as typeof output;
  assert.equal(decoded.transaction.status, "pending");
  assert.ok(decoded.transaction.buyer instanceof Uint8Array);
});

test("transaction resolution preserves safety projection evidence", () => {
  const confirmedHead = {
    transaction_hash: hash(82),
    transaction: {
      buyer: hash(83),
      seller: hash(84),
      listing_hash: hash(85),
      quantity: 1,
      total_price_cents: 1999,
      status: "confirmed",
      created_at: 1,
      updated_at: 2,
      tracking_info: null,
      epistemic: {
        empirical: "E1Testimonial",
        normative: "N1Communal",
        materiality: "M1Temporal",
      },
    },
  };
  const cancelledHead = {
    transaction_hash: hash(86),
    transaction: {
      ...confirmedHead.transaction,
      status: "cancelled",
      updated_at: 3,
    },
  };
  const resolution = {
    root_transaction_hash: hash(81),
    policy_version: 2,
    state: "auto_resolved",
    reason: "cancellation_dominates_pre_shipment",
    canonical: cancelledHead,
    heads: [confirmedHead, cancelledHead],
    superseded_heads: [confirmedHead],
    applied_conflict_resolutions: [],
    revision_count: 3,
  };

  const decoded = decodeZomePayload(
    encodeZomeResponse(resolution),
  ) as typeof resolution;
  assert.equal(decoded.state, "auto_resolved");
  assert.equal(decoded.policy_version, 2);
  assert.equal(decoded.reason, "cancellation_dominates_pre_shipment");
  assert.ok(decoded.root_transaction_hash instanceof Uint8Array);
  assert.ok(decoded.canonical?.transaction_hash instanceof Uint8Array);
  assert.ok(decoded.superseded_heads[0]?.transaction_hash instanceof Uint8Array);
  assert.equal(decoded.heads.length, 2);
});

test("explicit conflict authority preserves exact binary bindings", () => {
  const approval = {
    approval_hash: hash(121),
    approval: {
      protocol_version: 1,
      root_transaction_hash: hash(122),
      head_hashes: [hash(123), hash(124)],
      selected_head_hash: hash(124),
      approver: hash(125),
      rationale: "Prefer the shipped branch",
      created_at: 10,
    },
  };
  const resolution = {
    resolution_hash: hash(126),
    resolution: {
      protocol_version: 1,
      root_transaction_hash: hash(122),
      head_hashes: [hash(123), hash(124)],
      selected_head_hash: hash(124),
      authority: {
        bilateral: {
          buyer_approval_hash: hash(121),
          seller_approval_hash: hash(127),
        },
      },
      summary: "Both parties selected the shipped branch",
      created_at: 11,
    },
  };
  const decodedApproval = decodeZomePayload(
    encodeZomeResponse(approval),
  ) as typeof approval;
  const decodedResolution = decodeZomePayload(
    encodeZomeResponse(resolution),
  ) as typeof resolution;
  assert.ok(decodedApproval.approval_hash instanceof Uint8Array);
  assert.ok(decodedApproval.approval.root_transaction_hash instanceof Uint8Array);
  assert.ok(decodedApproval.approval.head_hashes.every((value) => value instanceof Uint8Array));
  assert.ok(decodedResolution.resolution.selected_head_hash instanceof Uint8Array);
  assert.ok(
    decodedResolution.resolution.authority.bilateral.buyer_approval_hash instanceof Uint8Array,
  );
});

test("dispute resolution preserves stable roots, exact revisions, and conflicts", () => {
  const resolution = {
    root_dispute_hash: hash(91),
    state: "conflicted",
    canonical: null,
    heads: [
      {
        dispute_hash: hash(92),
        dispute: {
          transaction_hash: hash(93),
          transaction_revision_hash: hash(94),
          conflict_heads: [hash(99), hash(100)],
          filed_by: hash(95),
          buyer: hash(95),
          seller: hash(96),
          reason: "Evidence mismatch",
          evidence_cids: [],
          status: "underreview",
          arbitrators: [hash(97)],
          result_hash: null,
          created_at: 1,
          updated_at: 2,
        },
      },
      {
        dispute_hash: hash(98),
        dispute: {
          transaction_hash: hash(93),
          transaction_revision_hash: hash(94),
          conflict_heads: [hash(99), hash(100)],
          filed_by: hash(95),
          buyer: hash(95),
          seller: hash(96),
          reason: "Evidence mismatch",
          evidence_cids: [],
          status: "withdrawn",
          arbitrators: [],
          result_hash: null,
          created_at: 1,
          updated_at: 2,
        },
      },
    ],
    revision_count: 3,
  };

  const decoded = decodeZomePayload(
    encodeZomeResponse(resolution),
  ) as typeof resolution;
  assert.equal(decoded.state, "conflicted");
  assert.ok(decoded.root_dispute_hash instanceof Uint8Array);
  assert.ok(decoded.heads[0]?.dispute.transaction_revision_hash instanceof Uint8Array);
  assert.ok(decoded.heads[0]?.dispute.arbitrators[0] instanceof Uint8Array);
  assert.equal(decoded.heads.length, 2);
});

test("arbitration result binds exact binary vote hashes and equal-weight ratio", () => {
  const result = {
    result_hash: hash(101),
    result: {
      dispute_hash: hash(102),
      dispute_revision_hash: hash(103),
      vote_hashes: [hash(104), hash(105), hash(106)],
      winner: hash(107),
      loser: hash(108),
      weighted_vote: 2 / 3,
      total_votes: 3,
      compensation_cents: 7500,
      summary: "Deterministic fixture result",
      finalized_at: 4,
    },
  };

  const decoded = decodeZomePayload(encodeZomeResponse(result)) as typeof result;
  assert.ok(decoded.result_hash instanceof Uint8Array);
  assert.ok(decoded.result.dispute_revision_hash instanceof Uint8Array);
  assert.equal(decoded.result.vote_hashes.length, 3);
  assert.ok(decoded.result.vote_hashes.every((value) => value instanceof Uint8Array));
  assert.equal(decoded.result.weighted_vote, 2 / 3);
});

test("settlement projection preserves stable root and Finance evidence", () => {
  const projection = {
    root_transaction_hash: hash(111),
    transaction_revision_hash: hash(112),
    state: "completed",
    settled: true,
    idempotency_reference: "marketplace_tx:uhCkk-fixture",
    finance_payment_id: "payment-fixture",
    finance_action_hash: hash(113),
    error: null,
  };

  const decoded = decodeZomePayload(
    encodeZomeResponse(projection),
  ) as typeof projection;
  assert.equal(decoded.state, "completed");
  assert.equal(decoded.settled, true);
  assert.ok(decoded.root_transaction_hash instanceof Uint8Array);
  assert.ok(decoded.transaction_revision_hash instanceof Uint8Array);
  assert.ok(decoded.finance_action_hash instanceof Uint8Array);
});

test("reputation events preserve exact evidence hashes and snake-case kinds", () => {
  const response = {
    events: [
      {
        event_hash: hash(121),
        event: {
          event_key: "fulfillment:fixture:seller:fixture",
          subject: hash(122),
          counterparty: hash(123),
          transaction_hash: hash(124),
          source_hash: hash(125),
          kind: "fulfillment_delivered",
          value_cents: 12500,
          occurred_at: 5,
        },
      },
    ],
  };

  const decoded = decodeZomePayload(encodeZomeResponse(response)) as typeof response;
  assert.equal(decoded.events[0]?.event.kind, "fulfillment_delivered");
  assert.ok(decoded.events[0]?.event_hash instanceof Uint8Array);
  assert.ok(decoded.events[0]?.event.transaction_hash instanceof Uint8Array);
  assert.ok(decoded.events[0]?.event.source_hash instanceof Uint8Array);
});
