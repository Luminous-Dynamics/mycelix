export class KnowledgeClientUnavailableError extends Error {
  constructor(message = 'Mycelix Knowledge integration is not available in this hApp bundle.') {
    super(message);
    this.name = 'KnowledgeClientUnavailableError';
  }
}

async function requireConfiguredKnowledgeRole(appClient, roleName) {
  if (!appClient || typeof appClient.appInfo !== 'function') {
    throw new KnowledgeClientUnavailableError(
      'Mycelix Knowledge requires a connected Holochain AppClient.'
    );
  }

  const appInfo = await appClient.appInfo();
  const role = appInfo?.cell_info?.[roleName];
  if (!role?.length) {
    throw new KnowledgeClientUnavailableError(
      `The optional Holochain role "${roleName}" is not installed in this hApp.`
    );
  }

  throw new KnowledgeClientUnavailableError(
    'A Knowledge role is installed, but this repository does not contain a verified Knowledge zome contract. Replace the local compatibility package with the published or workspace Mycelix Knowledge client before enabling this feature.'
  );
}

export class KnowledgeClient {
  constructor(appClient, roleName = 'knowledge') {
    this.appClient = appClient;
    this.roleName = roleName;

    this.query = {
      search: async () => requireConfiguredKnowledgeRole(this.appClient, this.roleName),
    };
    this.graph = {
      getDependencyTree: async () => requireConfiguredKnowledgeRole(this.appClient, this.roleName),
    };
    this.inference = {
      calculateEnhancedCredibility: async () =>
        requireConfiguredKnowledgeRole(this.appClient, this.roleName),
      getAuthorReputation: async () => requireConfiguredKnowledgeRole(this.appClient, this.roleName),
    };
    this.marketsIntegration = {
      requestVerificationMarket: async () =>
        requireConfiguredKnowledgeRole(this.appClient, this.roleName),
    };
  }
}

export class KnowledgeService {
  constructor(appClient, roleName = 'knowledge') {
    this.appClient = appClient;
    this.roleName = roleName;
  }

  async submitAndAnalyzeClaim() {
    return requireConfiguredKnowledgeRole(this.appClient, this.roleName);
  }
}

export function calculateInformationValue(uncertainty, dependentCount, averageDependencyWeight) {
  const u = clamp01(uncertainty);
  const count = Math.max(0, Number(dependentCount) || 0);
  const weight = Math.max(0, Number(averageDependencyWeight) || 0);
  return u * Math.log1p(count) * weight;
}

export function recommendVerification(claim, informationValue) {
  const empirical = clamp01(claim?.classification?.empirical ?? 0);
  const expectedValue = Math.max(0, Number(informationValue?.expectedValue) || 0);
  const recommend = empirical < 0.7 || expectedValue >= 0.25;

  return {
    recommend,
    reason: recommend
      ? 'The claim remains materially uncertain or affects dependent knowledge.'
      : 'The claim has sufficient empirical support for the current context.',
    suggestedTargetE: Math.max(0.7, Math.min(0.95, empirical + 0.2)),
  };
}

export function toDiscreteEpistemic(position) {
  return {
    empirical: classifyAxis(position?.empirical),
    normative: classifyAxis(position?.normative),
    mythic: classifyAxis(position?.mythic),
  };
}

function classifyAxis(value) {
  const score = clamp01(value ?? 0);
  if (score >= 0.75) return 'high';
  if (score >= 0.4) return 'medium';
  return 'low';
}

function clamp01(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return 0;
  return Math.max(0, Math.min(1, number));
}
