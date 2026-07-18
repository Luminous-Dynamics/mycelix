import assert from "node:assert/strict";
import {
  AdminWebsocket,
  AppWebsocket,
  type ActionHash,
  type AgentPubKey,
  type AppWebsocketConnectionOptions,
} from "@holochain/client";
import { emitLiveEvidence, requireActiveRoles } from "./evidence";

const ROLE = "marketplace";
const LISTINGS = "listings";
const TRANSACTIONS = "transactions";
const ARBITRATION = "arbitration";
const REPUTATION = "reputation";

type ListingOutput = {
  listing_hash: ActionHash;
  seller_agent_id: AgentPubKey;
  listing: { price_cents: number; status: string };
};

type TransactionOutput = {
  transaction_hash: ActionHash;
  transaction: {
    buyer: AgentPubKey;
    seller: AgentPubKey;
    total_price_cents: number;
    status: string;
  };
};

type OpenDisputeOutput = {
  transaction: TransactionOutput;
  dispute_hash: ActionHash;
};

type DisputeOutput = {
  dispute_hash: ActionHash;
  dispute: {
    transaction_hash: ActionHash;
    transaction_revision_hash: ActionHash;
    filed_by: AgentPubKey;
    buyer: AgentPubKey;
    seller: AgentPubKey;
    status: string;
    arbitrators: AgentPubKey[];
    result_hash: ActionHash | null;
  };
};

type DisputeResolution = {
  root_dispute_hash: ActionHash;
  state: "resolved" | "conflicted";
  canonical: DisputeOutput | null;
  heads: DisputeOutput[];
  revision_count: number;
};

type DisputesResponse = { disputes: DisputeOutput[] };

type ArbitrationVoteOutput = {
  vote_hash: ActionHash;
  vote: {
    dispute_hash: ActionHash;
    dispute_revision_hash: ActionHash;
    arbitrator: AgentPubKey;
    favor_buyer: boolean;
    arbitrator_matl_score: number;
  };
};

type ArbitrationResultOutput = {
  result_hash: ActionHash;
  result: {
    dispute_hash: ActionHash;
    dispute_revision_hash: ActionHash;
    vote_hashes: ActionHash[];
    winner: AgentPubKey;
    loser: AgentPubKey;
    weighted_vote: number;
    total_votes: number;
    compensation_cents: number | null;
  };
};

type IdentityConfig = {
  label: string;
  appUrl: URL;
  adminUrl: URL;
  token: number[];
};

type ConnectedIdentity = {
  app: AppWebsocket;
  admin: AdminWebsocket;
};

function required(name: string): string {
  const value = process.env[name];
  if (!value) throw new Error(`missing required environment variable ${name}`);
  return value;
}

function decodeToken(value: string): number[] {
  const normalized = value.replace(/-/g, "+").replace(/_/g, "/");
  return Array.from(Buffer.from(normalized, "base64"));
}

function identity(prefix: "SELLER" | "BUYER" | "ARBITRATOR"): IdentityConfig {
  return {
    label: prefix.toLowerCase(),
    appUrl: new URL(required(`MARKETPLACE_${prefix}_APP_URL`)),
    adminUrl: new URL(required(`MARKETPLACE_${prefix}_ADMIN_URL`)),
    token: decodeToken(required(`MARKETPLACE_${prefix}_TOKEN_BASE64`)),
  };
}

async function connectIdentity(config: IdentityConfig): Promise<ConnectedIdentity> {
  const options: AppWebsocketConnectionOptions = {
    url: config.appUrl,
    token: config.token,
    defaultTimeout: 30_000,
  };
  const app = await AppWebsocket.connect(options);
  const admin = await AdminWebsocket.connect({ url: config.adminUrl });
  const info = await app.appInfo();
  const cellId = app.getCellIdFromRoleName(ROLE, info);
  await admin.authorizeSigningCredentials(cellId);
  console.log(`${config.label}: connected and authorized signing credentials`);
  return { app, admin };
}

async function call<T>(
  app: AppWebsocket,
  zome: string,
  fnName: string,
  payload: unknown,
): Promise<T> {
  return app.callZome<T>({
    role_name: ROLE,
    zome_name: zome,
    fn_name: fnName,
    payload,
  });
}

async function eventually<T>(
  label: string,
  operation: () => Promise<T>,
  accept: (value: T) => boolean,
): Promise<T> {
  const deadline = Date.now() + 60_000;
  let lastError: unknown;
  while (Date.now() < deadline) {
    try {
      const value = await operation();
      if (accept(value)) return value;
      lastError = new Error(`${label}: returned a value outside the expected state`);
    } catch (error) {
      lastError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 750));
  }
  throw new Error(`${label}: timed out`, { cause: lastError });
}

async function score(app: AppWebsocket, agent: AgentPubKey): Promise<unknown> {
  return call<unknown>(app, REPUTATION, "get_agent_matl_score", agent);
}

async function main(): Promise<void> {
  const seller = await connectIdentity(identity("SELLER"));
  const buyer = await connectIdentity(identity("BUYER"));
  const arbitrator = await connectIdentity(identity("ARBITRATOR"));
  const activeRoleSets = {
    seller: await requireActiveRoles(seller.app, [ROLE]),
    buyer: await requireActiveRoles(buyer.app, [ROLE]),
    arbitrator: await requireActiveRoles(arbitrator.app, [ROLE]),
  };

  try {
    assert.notDeepEqual(Array.from(seller.app.myPubKey), Array.from(buyer.app.myPubKey));
    assert.notDeepEqual(Array.from(arbitrator.app.myPubKey), Array.from(buyer.app.myPubKey));
    assert.notDeepEqual(Array.from(arbitrator.app.myPubKey), Array.from(seller.app.myPubKey));

    await call<void>(arbitrator.app, ARBITRATION, "register_as_arbitrator", null);

    const listing = await call<ListingOutput>(seller.app, LISTINGS, "create_listing", {
      title: `Arbitration evidence fixture ${Date.now()}`,
      description:
        "Conductor-backed evidence listing for conflict-aware, equal-weight arbitration.",
      price_cents: 10_000,
      category: "Electronics",
      photos_ipfs_cids: [],
      quantity_available: 1,
    });
    assert.equal(listing.listing.status, "active");

    await eventually(
      "listing propagation",
      () => call<ListingOutput | null>(buyer.app, LISTINGS, "get_listing", listing.listing_hash),
      (value) => value !== null,
    );

    const pending = await call<TransactionOutput>(
      buyer.app,
      TRANSACTIONS,
      "create_transaction",
      {
        seller: listing.seller_agent_id,
        listing_hash: listing.listing_hash,
        quantity: 1,
        total_price_cents: listing.listing.price_cents,
      },
    );

    const buyerScoreBefore = await score(buyer.app, buyer.app.myPubKey);
    const sellerScoreBefore = await score(buyer.app, seller.app.myPubKey);

    const opened = await eventually(
      "open dispute after arbitrator registration propagation",
      () =>
        call<OpenDisputeOutput>(buyer.app, TRANSACTIONS, "open_dispute", {
          transaction_hash: pending.transaction_hash,
          reason: "The delivered evidence does not match the listing claim.",
          evidence_cids: [],
        }),
      (value) => value.transaction.transaction.status === "disputed",
    );
    const root = opened.dispute_hash;

    const assigned = await eventually(
      "assigned dispute propagation",
      () =>
        call<DisputeResolution | null>(
          arbitrator.app,
          ARBITRATION,
          "get_dispute_resolution",
          root,
        ),
      (value): value is DisputeResolution =>
        value !== null &&
        value.state === "resolved" &&
        value.canonical?.dispute.status === "underreview" &&
        value.canonical.dispute.arbitrators.some((agent) =>
          Buffer.from(agent).equals(Buffer.from(arbitrator.app.myPubKey)),
        ),
    );
    assert.deepEqual(Array.from(assigned.root_dispute_hash), Array.from(root));

    const opportunities = await eventually(
      "arbitration opportunity propagation",
      () =>
        call<DisputesResponse>(
          arbitrator.app,
          ARBITRATION,
          "get_arbitration_opportunities",
          null,
        ),
      (value) => value.disputes.length === 1,
    );
    assert.equal(opportunities.disputes.length, 1);

    const vote = await call<ArbitrationVoteOutput>(
      arbitrator.app,
      ARBITRATION,
      "submit_arbitration_vote",
      {
        dispute_hash: root,
        favor_buyer: true,
        reasoning: "The case evidence supports the buyer claim.",
      },
    );
    assert.equal(vote.vote.arbitrator_matl_score, 1.0);
    assert.deepEqual(Array.from(vote.vote.dispute_hash), Array.from(root));

    await assert.rejects(() =>
      call<ArbitrationVoteOutput>(
        arbitrator.app,
        ARBITRATION,
        "submit_arbitration_vote",
        {
          dispute_hash: root,
          favor_buyer: false,
          reasoning: "Duplicate vote must fail.",
        },
      ),
    );

    await eventually(
      "voting revision propagation",
      () =>
        call<DisputeResolution | null>(
          arbitrator.app,
          ARBITRATION,
          "get_dispute_resolution",
          root,
        ),
      (value) => value?.canonical?.dispute.status === "voting",
    );

    const finalized = await call<ArbitrationResultOutput>(
      arbitrator.app,
      ARBITRATION,
      "finalize_arbitration",
      root,
    );
    assert.deepEqual(Array.from(finalized.result.dispute_hash), Array.from(root));
    assert.equal(finalized.result.total_votes, 1);
    assert.equal(finalized.result.weighted_vote, 1.0);
    assert.equal(finalized.result.vote_hashes.length, 1);
    assert.deepEqual(
      Array.from(finalized.result.vote_hashes[0]!),
      Array.from(vote.vote_hash),
    );
    assert.deepEqual(Array.from(finalized.result.winner), Array.from(buyer.app.myPubKey));
    assert.equal(finalized.result.compensation_cents, 10_000);

    const finalizedAgain = await call<ArbitrationResultOutput>(
      arbitrator.app,
      ARBITRATION,
      "finalize_arbitration",
      root,
    );
    assert.deepEqual(
      Array.from(finalizedAgain.result_hash),
      Array.from(finalized.result_hash),
    );

    const observedResult = await eventually(
      "result propagation",
      () =>
        call<ArbitrationResultOutput | null>(
          buyer.app,
          ARBITRATION,
          "get_arbitration_result",
          root,
        ),
      (value): value is ArbitrationResultOutput => value !== null,
    );
    assert.deepEqual(Array.from(observedResult.result_hash), Array.from(finalized.result_hash));

    const resolved = await eventually(
      "resolved dispute propagation",
      () =>
        call<DisputeResolution | null>(
          buyer.app,
          ARBITRATION,
          "get_dispute_resolution",
          root,
        ),
      (value): value is DisputeResolution =>
        value?.state === "resolved" && value.canonical?.dispute.status === "resolvedbuyer",
    );
    assert.equal(resolved.heads.length, 1);

    const buyerScoreAfter = await score(buyer.app, buyer.app.myPubKey);
    const sellerScoreAfter = await score(buyer.app, seller.app.myPubKey);
    assert.deepEqual(buyerScoreAfter, buyerScoreBefore);
    assert.deepEqual(sellerScoreAfter, sellerScoreBefore);

    await emitLiveEvidence(
      "arbitration",
      activeRoleSets,
      {
        dispute_root: Array.from(root),
        revisions: resolved.revision_count,
        vote_weight: vote.vote.arbitrator_matl_score,
        result_vote_count: finalized.result.vote_hashes.length,
        reputation_unchanged: true,
        duplicate_vote_rejected: true,
        idempotent_finalization: true,
        conflict_injection_tested: false,
      },
    );
  } finally {
    await Promise.allSettled([
      seller.app.client.close(),
      seller.admin.client.close(),
      buyer.app.client.close(),
      buyer.admin.client.close(),
      arbitrator.app.client.close(),
      arbitrator.admin.client.close(),
    ]);
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
