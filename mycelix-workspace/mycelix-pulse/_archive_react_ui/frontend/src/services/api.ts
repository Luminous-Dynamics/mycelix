// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  User,
  Email,
  EmailAccount,
  Folder,
  LoginCredentials,
  RegisterData,
  ApiResponse,
} from '@/types';
import { config } from '@/config/env';
import { errorLogger } from './errorLogger';

const API_URL = config.apiUrl;

class ApiService {
  public client: AxiosInstance;

  constructor() {
    this.client = axios.create({
      baseURL: API_URL,
      headers: {
        'Content-Type': 'application/json',
      },
    });

    // Add auth token to requests
    this.client.interceptors.request.use((config) => {
      const token = localStorage.getItem('token');
      if (token) {
        config.headers.Authorization = `Bearer ${token}`;
      }
      return config;
    });

    // Handle errors and log them
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        const status = error.response?.status;
        const url = error.config?.url;
        const method = error.config?.method?.toUpperCase();

        // Log API errors
        errorLogger.logError(error, {
          action: 'api_request_failed',
          metadata: {
            url,
            method,
            status,
            statusText: error.response?.statusText,
            data: error.response?.data,
          },
        }, status === 500 ? 'high' : 'medium');

        // Handle 401 errors (unauthorized)
        if (status === 401) {
          localStorage.removeItem('token');
          window.location.href = '/login';
        }

        return Promise.reject(error);
      }
    );
  }

  // Auth endpoints
  async login(credentials: LoginCredentials) {
    const { data } = await this.client.post<ApiResponse<{ user: User; token: string }>>(
      '/api/auth/login',
      credentials
    );
    return data.data;
  }

  async register(userData: RegisterData) {
    const { data } = await this.client.post<ApiResponse<{ user: User; token: string }>>(
      '/api/auth/register',
      userData
    );
    return data.data;
  }

  async getProfile() {
    const { data } = await this.client.get<ApiResponse<{ user: User }>>('/api/auth/me');
    return data.data.user;
  }

  async updateProfile(updates: Partial<User>) {
    const { data } = await this.client.put<ApiResponse<{ user: User }>>('/api/auth/me', updates);
    return data.data.user;
  }

  async changePassword(currentPassword: string, newPassword: string) {
    const { data } = await this.client.put<ApiResponse<{ message: string }>>(
      '/api/auth/password',
      { currentPassword, newPassword }
    );
    return data.data;
  }

  // Email Account endpoints
  async getAccounts() {
    const { data } = await this.client.get<ApiResponse<{ accounts: EmailAccount[] }>>(
      '/api/accounts'
    );
    return data.data.accounts;
  }

  async createAccount(accountData: Partial<EmailAccount> & { imapPassword: string; smtpPassword: string }) {
    const { data } = await this.client.post<ApiResponse<{ account: EmailAccount }>>(
      '/api/accounts',
      accountData
    );
    return data.data.account;
  }

  async updateAccount(accountId: string, updates: Partial<EmailAccount>) {
    const { data } = await this.client.put<ApiResponse<{ account: EmailAccount }>>(
      `/api/accounts/${accountId}`,
      updates
    );
    return data.data.account;
  }

  async deleteAccount(accountId: string) {
    const { data } = await this.client.delete<ApiResponse<{ message: string }>>(
      `/api/accounts/${accountId}`
    );
    return data.data;
  }

  async setDefaultAccount(accountId: string) {
    const { data } = await this.client.put<ApiResponse<{ account: EmailAccount }>>(
      `/api/accounts/${accountId}/default`
    );
    return data.data.account;
  }

  // Folder endpoints
  async getFolders(accountId?: string) {
    const { data } = await this.client.get<ApiResponse<{ folders: Folder[] }>>(
      '/api/folders',
      { params: { accountId } }
    );
    return data.data.folders;
  }

  async createFolder(folderData: { emailAccountId: string; name: string; path: string }) {
    const { data } = await this.client.post<ApiResponse<{ folder: Folder }>>(
      '/api/folders',
      folderData
    );
    return data.data.folder;
  }

  async deleteFolder(folderId: string) {
    const { data } = await this.client.delete<ApiResponse<{ message: string }>>(
      `/api/folders/${folderId}`
    );
    return data.data;
  }

  // Email endpoints
  async getEmails(params?: {
    accountId?: string;
    folderId?: string;
    page?: number;
    limit?: number;
    search?: string;
  }) {
    const { data } = await this.client.get<
      ApiResponse<{ emails: Email[]; pagination: any }>
    >('/api/emails', { params });
    return data.data;
  }

  async getEmail(emailId: string) {
    const { data } = await this.client.get<ApiResponse<{ email: Email }>>(
      `/api/emails/${emailId}`
    );
    return data.data.email;
  }

  async sendEmail(emailData: {
    accountId: string;
    to: string[];
    subject: string;
    body: string;
    cc?: string[];
    bcc?: string[];
    attachments?: any[];
  }) {
    const { data } = await this.client.post<ApiResponse<{ messageId: string }>>(
      '/api/emails',
      emailData
    );
    return data.data;
  }

  async markEmailRead(emailId: string, isRead: boolean) {
    const { data } = await this.client.put<ApiResponse<{ email: Email }>>(
      `/api/emails/${emailId}/read`,
      { isRead }
    );
    return data.data.email;
  }

  async markEmailStarred(emailId: string, isStarred: boolean) {
    const { data } = await this.client.put<ApiResponse<{ email: Email }>>(
      `/api/emails/${emailId}/star`,
      { isStarred }
    );
    return data.data.email;
  }

  async deleteEmail(emailId: string) {
    const { data } = await this.client.delete<ApiResponse<{ message: string }>>(
      `/api/emails/${emailId}`
    );
    return data.data;
  }

  async syncEmails(accountId: string, folderPath: string = 'INBOX') {
    const { data } = await this.client.post<ApiResponse<{ synced: number }>>(
      '/api/emails/sync',
      { accountId, folderPath }
    );
    return data.data;
  }

  // Trust / MATL endpoints (optional; no-op if backend does not expose)
  async getTrustSummary(sender: string) {
    const { data } = await this.client.get<
      ApiResponse<{
        summary: {
          score?: number;
          tier?: 'high' | 'medium' | 'low' | 'unknown';
          reasons?: string[];
          pathLength?: number;
          decayAt?: string;
          quarantined?: boolean;
          attestations?: { from: string; to: string; weight: number; reason?: string }[];
          fetchedAt?: string;
        };
      }>
    >('/api/trust/summary', { params: { sender } });

    return data.data.summary;
  }

  async getTrustHealth() {
    const { data } = await this.client.get<
      ApiResponse<{ providerConfigured: boolean; cacheSize: number; ttlMs: number }>
    >('/api/trust/health');
    return data.data;
  }

  // ============================================
  // Claims API (0TML Integration)
  // ============================================

  async verifyClaims(claims: VerifiableClaim[], assuranceLevel?: AssuranceLevel) {
    const { data } = await this.client.post<ApiResponse<VerifyClaimsResponse>>(
      '/api/claims/verify',
      { claims, assurance_level: assuranceLevel }
    );
    return data.data;
  }

  async listCredentials() {
    const { data } = await this.client.get<ApiResponse<CredentialsListResponse>>(
      '/api/claims/credentials'
    );
    return data.data;
  }

  async getCredential(id: string) {
    const { data } = await this.client.get<ApiResponse<{ credential: VerifiableCredential }>>(
      `/api/claims/credentials/${id}`
    );
    return data.data.credential;
  }

  async attachClaim(claim: VerifiableClaim, proofType: ProofType) {
    const { data } = await this.client.post<ApiResponse<AttachClaimResponse>>(
      '/api/claims/attach',
      { claim, proof_type: proofType }
    );
    return data.data;
  }

  async getAssuranceLevel(did: string) {
    const { data } = await this.client.get<ApiResponse<AssuranceLevelResponse>>(
      `/api/claims/assurance/${did}`
    );
    return data.data;
  }

  // ============================================
  // AI API (Symthaea Integration)
  // ============================================

  async analyzeEmail(email: Email) {
    const { data } = await this.client.post<ApiResponse<AIInsights>>(
      '/api/ai/analyze',
      { email }
    );
    return data.data;
  }

  async summarizeThread(emails: Email[]) {
    const { data } = await this.client.post<ApiResponse<ThreadSummary>>(
      '/api/ai/summarize',
      { emails }
    );
    return data.data;
  }

  async detectIntent(email: Email) {
    const { data } = await this.client.post<ApiResponse<DetectIntentResponse>>(
      '/api/ai/intent',
      { email }
    );
    return data.data;
  }

  async suggestReplies(email: Email) {
    const { data } = await this.client.post<ApiResponse<SuggestRepliesResponse>>(
      '/api/ai/suggest-reply',
      { email }
    );
    return data.data;
  }

  async explainTrust(senderDid: string, trustScore: number, trustPath: [string, string, number][]) {
    const { data } = await this.client.post<ApiResponse<TrustExplanation>>(
      '/api/ai/explain-trust',
      { sender_did: senderDid, trust_score: trustScore, trust_path: trustPath }
    );
    return data.data;
  }

  async getAIStatus() {
    const { data } = await this.client.get<ApiResponse<ConsciousnessState>>(
      '/api/ai/status'
    );
    return data.data;
  }

  // ============================================
  // Trust Graph API
  // ============================================

  async getTrustGraph(did: string, depth: number = 2) {
    const { data } = await this.client.get<ApiResponse<TrustGraph>>(
      `/api/trust-graph/graph/${did}`,
      { params: { depth } }
    );
    return data.data;
  }

  async findTrustPath(fromDid: string, toDid: string) {
    const { data } = await this.client.get<ApiResponse<TrustPath>>(
      `/api/trust-graph/path/${fromDid}/${toDid}`
    );
    return data.data;
  }

  async createAttestation(request: CreateAttestationRequest) {
    const { data } = await this.client.post<ApiResponse<CreateAttestationResponse>>(
      '/api/trust-graph/attest',
      request
    );
    return data.data;
  }

  async revokeAttestation(id: string) {
    const { data } = await this.client.delete<ApiResponse<{ revoked: boolean; id: string }>>(
      `/api/trust-graph/attest/${id}`
    );
    return data.data;
  }

  async getTrustedBy(did: string) {
    const { data } = await this.client.get<ApiResponse<TrustEdgesResponse>>(
      `/api/trust-graph/trusted-by/${did}`
    );
    return data.data;
  }

  async getTrusters(did: string) {
    const { data } = await this.client.get<ApiResponse<TrustEdgesResponse>>(
      `/api/trust-graph/trusters/${did}`
    );
    return data.data;
  }
}

// ============================================
// Type definitions for new APIs
// ============================================

// Claims types
export type AssuranceLevel = 'e0_anonymous' | 'e1_verified_email' | 'e2_gitcoin_passport' | 'e3_multi_factor' | 'e4_constitutional';
export type ProofType = 'risc0_zk_vm' | 'winterfell_air' | 'simplified_hash' | 'ed25519_signature' | 'none';
export type CredentialType = 'verified_human' | 'gitcoin_passport' | 'reputation_score' | 'employment' | 'education' | 'certification' | 'membership' | 'attestation';

export interface VerifiableCredential {
  id: string;
  credential_type: CredentialType;
  issuer_did: string;
  subject_did: string;
  issued_at: string;
  expires_at?: string;
  claims: Record<string, unknown>;
}

export interface VerifiableClaim {
  claim_type: 'identity' | 'affiliation' | 'credential' | 'cryptographic_proof';
  claim?: string;
  organization?: string;
  role?: string;
  statement?: string;
  proof_type?: ProofType;
  credential?: VerifiableCredential;
}

export interface ClaimVerification {
  verified: boolean;
  epistemic_tier: number;
  proof_type: ProofType;
  verified_at: string;
}

export interface VerifyClaimsResponse {
  verifications: ClaimVerification[];
  overall_epistemic_tier: number;
  all_verified: boolean;
}

export interface CredentialsListResponse {
  credentials: VerifiableCredential[];
  assurance_level: AssuranceLevel;
}

export interface AttachClaimResponse {
  claim: VerifiableClaim;
  proof: string;
  epistemic_tier: number;
}

export interface AssuranceLevelResponse {
  did: string;
  level: AssuranceLevel;
  attack_cost_usd: number;
  trust_multiplier: number;
}

// AI types
export type EmailIntent = 'meeting_request' | 'action_item' | 'fyi' | 'question' | 'social_greeting' | 'commercial' | 'spam' | 'unknown';

export interface AIInsights {
  intent: EmailIntent;
  sentiment: 'positive' | 'negative' | 'neutral' | 'mixed';
  urgency: 'high' | 'medium' | 'low' | 'none';
  key_topics: string[];
  suggested_labels: string[];
  reply_needed: boolean;
  confidence: number;
}

export interface ThreadSummary {
  summary: string;
  key_points: string[];
  action_items: string[];
  participants_summary: Record<string, string>;
  sentiment_arc: ('positive' | 'negative' | 'neutral')[];
  thread_health: 'active' | 'stale' | 'resolved' | 'needs_attention';
}

export interface ReplySuggestion {
  tone: 'formal' | 'casual' | 'friendly' | 'professional';
  body: string;
  confidence: number;
}

export interface DetectIntentResponse {
  intent: EmailIntent;
  suggested_action: string;
  priority_weight: number;
}

export interface SuggestRepliesResponse {
  suggestions: ReplySuggestion[];
}

export interface TrustExplanation {
  summary: string;
  factors: { factor: string; contribution: number; explanation: string }[];
  recommendations: string[];
  confidence: number;
}

export interface ConsciousnessState {
  active: boolean;
  model_loaded: boolean;
  semantic_dimensions: number;
  ltc_neurons: number;
  inference_mode: 'local' | 'hybrid' | 'disabled';
  last_inference_ms?: number;
  safety_score: number;
}

// Trust Graph types
export type RelationType = 'personal_contact' | 'professional_colleague' | 'organization_member' | 'verified_service' | 'community_member' | 'transitive';

export interface TrustNode {
  did: string;
  display_name?: string;
  assurance_level: AssuranceLevel;
  node_type: 'person' | 'organization' | 'service' | 'unknown';
}

export interface TrustEdge {
  from_did: string;
  to_did: string;
  relationship: RelationType;
  weight: number;
  created_at: string;
  expires_at?: string;
}

export interface TrustHop {
  from_did: string;
  to_did: string;
  relationship: RelationType;
  weight: number;
}

export interface TrustPath {
  hops: TrustHop[];
  total_weight: number;
  path_length: number;
}

export interface TrustGraph {
  nodes: TrustNode[];
  edges: TrustEdge[];
  center_did: string;
  depth: number;
}

export interface TrustAttestation {
  id: string;
  attestor_did: string;
  subject_did: string;
  message: string;
  relationship: RelationType;
  trust_score: number;
  created_at: string;
  expires_at?: string;
}

export interface CreateAttestationRequest {
  subject_did: string;
  message: string;
  relationship: RelationType;
  trust_score: number;
  expires_days?: number;
}

export interface CreateAttestationResponse {
  attestation: TrustAttestation;
}

export interface TrustEdgesResponse {
  edges: TrustEdge[];
}

export const api = new ApiService();
