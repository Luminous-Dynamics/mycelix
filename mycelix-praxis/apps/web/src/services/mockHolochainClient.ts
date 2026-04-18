// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Mock Holochain Client
 *
 * Simulates Holochain conductor connection and zome function calls for development and demo purposes.
 * Returns realistic mock data with simulated latency and occasional errors.
 */

// Import example data
import courseExample1 from '../../../../examples/courses/spanish-beginner.json';
// import courseExample2 from '../../../../examples/courses/rust-fundamentals.json';
// import flRoundActive from '../../../../examples/fl-rounds/round-002-active.json';
import flRoundCompleted from '../../../../examples/fl-rounds/round-001-completed.json';
import credentialExample from '../../../../examples/credentials/valid-achievement.json';

export interface ZomeCallParams {
  zome: string;
  fnName: string;
  payload: any;
}

export interface MockHolochainClientConfig {
  /** Minimum latency in ms */
  minLatency?: number;
  /** Maximum latency in ms */
  maxLatency?: number;
  /** Probability of error (0.0 to 1.0) */
  errorRate?: number;
  /** Enable console logging */
  verbose?: boolean;
}

const DEFAULT_CONFIG: MockHolochainClientConfig = {
  minLatency: 100,
  maxLatency: 300,
  errorRate: 0.1,
  verbose: true,
};

export class MockHolochainClient {
  private config: MockHolochainClientConfig;
  private connected: boolean = false;

  // In-memory state
  private courses: any[] = [];
  private flRounds: any[] = [];
  private credentials: any[] = [];
  private enrollments: Map<string, string[]> = new Map(); // agentId -> courseIds

  constructor(config: Partial<MockHolochainClientConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.initializeMockData();
  }

  /**
   * Initialize mock data from examples
   */
  private initializeMockData() {
    // Load courses
    this.courses = [
      courseExample1,
      this.generateMockCourse('rust-fundamentals', 'Rust Fundamentals'),
      this.generateMockCourse('machine-learning-intro', 'Machine Learning Fundamentals'),
      this.generateMockCourse('web3-dev', 'Web3 Development Basics'),
      this.generateMockCourse('data-structures', 'Advanced Data Structures'),
      this.generateMockCourse('cryptography', 'Applied Cryptography'),
    ];

    // Load FL rounds
    this.flRounds = [
      flRoundCompleted,
      this.generateMockFlRound('fl-round-002', 'UPDATE', 8),
      this.generateMockFlRound('fl-round-003', 'DISCOVER', 0),
      this.generateMockFlRound('fl-round-004', 'JOIN', 5),
    ];

    // Load credentials
    this.credentials = [
      credentialExample,
      this.generateMockCredential('rust-fundamentals-2025'),
      this.generateMockCredential('spanish-beginner-2025'),
    ];

    if (this.config.verbose) {
      console.log('[MockHolochainClient] Initialized with mock data');
      console.log(`  - ${this.courses.length} courses`);
      console.log(`  - ${this.flRounds.length} FL rounds`);
      console.log(`  - ${this.credentials.length} credentials`);
    }
  }

  /**
   * Simulate connection to conductor
   */
  async connect(): Promise<void> {
    await this.simulateLatency();

    if (Math.random() < this.config.errorRate!) {
      throw new Error('Failed to connect to conductor');
    }

    this.connected = true;

    if (this.config.verbose) {
      console.log('[MockHolochainClient] Connected to conductor');
    }
  }

  /**
   * Disconnect from conductor
   */
  disconnect(): void {
    this.connected = false;

    if (this.config.verbose) {
      console.log('[MockHolochainClient] Disconnected from conductor');
    }
  }

  /**
   * Check connection status
   */
  isConnected(): boolean {
    return this.connected;
  }

  /**
   * Call a zome function
   */
  async callZome(zome: string, fnName: string, payload: any = {}): Promise<any> {
    if (!this.connected) {
      throw new Error('Not connected to conductor. Call connect() first.');
    }

    await this.simulateLatency();

    // Simulate occasional errors
    if (Math.random() < this.config.errorRate!) {
      throw new Error(`Zome call failed: ${zome}::${fnName}`);
    }

    if (this.config.verbose) {
      console.log(`[MockHolochainClient] Calling ${zome}::${fnName}`, payload);
    }

    // Route to appropriate handler
    switch (zome) {
      case 'learning_coordinator':
        return this.handleLearningZome(fnName, payload);
      case 'fl_coordinator':
        return this.handleFlZome(fnName, payload);
      case 'credential_coordinator':
        return this.handleCredentialZome(fnName, payload);
      case 'dao_coordinator':
        return this.handleDaoZome(fnName, payload);
      case 'srs_coordinator':
        return this.handleSrsZome(fnName, payload);
      case 'gamification_coordinator':
        return this.handleGamificationZome(fnName, payload);
      case 'adaptive_coordinator':
        return this.handleAdaptiveZome(fnName, payload);
      case 'integration_coordinator':
        return this.handleIntegrationZome(fnName, payload);
      case 'mentorship_coordinator':
        return this.handleMentorshipZome(fnName, payload);
      case 'praxis_bridge':
        return this.handleBridgeZome(fnName, payload);
      default:
        throw new Error(`Unknown zome: ${zome}`);
    }
  }

  /**
   * Handle mentorship zome functions
   */
  private handleMentorshipZome(fnName: string, _payload: any): any {
    switch (fnName) {
      case 'register_as_mentor':
      case 'register_as_mentee':
      case 'propose_mentorship':
      case 'accept_mentorship':
      case 'record_session':
      case 'complete_mentorship':
        return this.mockActionHash();
      case 'get_mentor':
        return this.wrapAsRecord({
          skills: ['Rust', 'WebAssembly'], experience_years: 5,
          bio: 'Experienced Rust developer', max_mentees: 3, active_mentees: 1,
          availability: 'Available', rating: 4.5, total_sessions: 12,
          created_at: Date.now() / 1000
        });
      case 'get_mentee':
        return this.wrapAsRecord({
          target_skills: ['Rust'], skill_level: 'beginner',
          goals: 'Learn systems programming', preferred_schedule: 'weekends',
          learning_pace: 'moderate', communication_preferences: ['async'],
          created_at: Date.now() / 1000
        });
      case 'find_mentors_by_skill':
      case 'get_all_mentors':
      case 'get_my_mentorships':
      case 'get_active_mentorships':
        return [];
      case 'get_mentorship_sessions':
        return [];
      default:
        return null;
    }
  }

  /**
   * Handle bridge zome functions
   */
  private handleBridgeZome(fnName: string, _payload: any): any {
    switch (fnName) {
      case 'dispatch_call':
        return { status: 'ok', result: null };
      case 'get_bridge_health':
        return {
          status: 'healthy', total_dispatches: 42,
          total_queries: 15, total_events: 28,
          uptime_seconds: 3600
        };
      case 'get_all_events':
      case 'get_domain_events':
      case 'get_my_events':
      case 'get_my_queries':
        return [];
      default:
        return null;
    }
  }

  /**
   * Handle learning zome functions
   */
  private handleLearningZome(fnName: string, payload: any): any {
    switch (fnName) {
      case 'list_courses':
        // Return as mock Record format
        return this.courses.map(c => this.wrapAsRecord(c));

      case 'get_course':
        const course = this.courses.find(c => c.course_id === payload?.course_id);
        if (!course) return null;
        return this.wrapAsRecord(course);

      case 'create_course':
        const newCourse = { ...payload, created_at: Date.now() / 1000 };
        this.courses.push(newCourse);
        return this.mockActionHash();

      case 'enroll':
        const agentId = 'mock-agent-123';
        const courseIds = this.enrollments.get(agentId) || [];
        if (!courseIds.includes(payload)) {
          courseIds.push(payload);
          this.enrollments.set(agentId, courseIds);
        }
        return undefined;

      case 'get_enrolled_courses':
        const agentEnrollments = this.enrollments.get('mock-agent-123') || [];
        return agentEnrollments.map(courseId =>
          this.courses.find(c => c.course_id === courseId)
        ).filter(Boolean).map(c => this.wrapAsRecord(c));

      case 'update_progress':
        return this.mockActionHash();

      case 'get_progress':
        return null;

      case 'record_activity':
        return this.mockActionHash();

      case 'get_course_enrollments':
        return [this.mockAgentPubKey()];

      default:
        throw new Error(`Unknown function: ${fnName}`);
    }
  }

  /**
   * Generate mock ActionHash
   */
  private mockActionHash(): Uint8Array {
    const hash = new Uint8Array(39);
    hash[0] = 132; // ActionHash prefix
    for (let i = 1; i < 39; i++) {
      hash[i] = Math.floor(Math.random() * 256);
    }
    return hash;
  }

  /**
   * Generate mock AgentPubKey
   */
  private mockAgentPubKey(): Uint8Array {
    const key = new Uint8Array(39);
    key[0] = 132; // AgentPubKey prefix
    for (let i = 1; i < 39; i++) {
      key[i] = Math.floor(Math.random() * 256);
    }
    return key;
  }

  /**
   * Wrap data as a Holochain Record
   */
  private wrapAsRecord(data: any): any {
    return {
      signed_action: {
        hashed: {
          content: {},
          hash: this.mockActionHash(),
        },
        signature: new Uint8Array(64),
      },
      entry: {
        Present: data,
      },
    };
  }

  /**
   * Handle FL zome functions
   */
  private handleFlZome(fnName: string, payload: any): any {
    switch (fnName) {
      case 'get_rounds':
        return this.flRounds;

      case 'get_round':
        const round = this.flRounds.find(r => r.round_id === payload.round_id);
        if (!round) throw new Error('Round not found');
        return round;

      case 'create_round':
        const newRound = this.generateMockFlRound(
          payload.round_id || `fl-round-${Date.now()}`,
          'DISCOVER',
          0
        );
        this.flRounds.push(newRound);
        return newRound;

      case 'join_round':
        const joinRound = this.flRounds.find(r => r.round_id === payload.round_id);
        if (!joinRound) throw new Error('Round not found');
        joinRound.current_participants = (joinRound.current_participants || 0) + 1;
        return { success: true, round_id: payload.round_id };

      case 'submit_update':
        return { success: true, update_hash: this.generateHash() };

      default:
        throw new Error(`Unknown function: ${fnName}`);
    }
  }

  /**
   * Handle credential zome functions
   */
  private handleCredentialZome(fnName: string, payload: any): any {
    switch (fnName) {
      case 'get_credentials':
        return this.credentials;

      case 'get_credential':
        const credential = this.credentials.find(c => c.id === payload.credential_id);
        if (!credential) throw new Error('Credential not found');
        return credential;

      case 'issue_credential':
        const newCredential = this.generateMockCredential(payload.course_id);
        this.credentials.push(newCredential);
        return newCredential;

      case 'verify_credential':
        // Simulate verification
        return {
          valid: true,
          issuer: 'did:key:z6Mk...',
          verified_at: new Date().toISOString()
        };

      default:
        throw new Error(`Unknown function: ${fnName}`);
    }
  }

  /**
   * Handle DAO zome functions
   */
  private handleDaoZome(fnName: string, payload: any): any {
    switch (fnName) {
      case 'get_proposals':
        return [
          {
            proposal_id: 'prop-001',
            title: 'Add Dark Mode',
            description: 'Implement dark mode theme',
            status: 'active',
            votes_for: 42,
            votes_against: 3,
          },
          {
            proposal_id: 'prop-002',
            title: 'New Course Category: Art',
            description: 'Add art courses to platform',
            status: 'passed',
            votes_for: 87,
            votes_against: 12,
          },
        ];

      case 'create_proposal':
        return {
          proposal_id: `prop-${Date.now()}`,
          ...payload,
          status: 'active',
          votes_for: 0,
          votes_against: 0,
        };

      case 'vote':
        return { success: true, proposal_id: payload.proposal_id };

      default:
        throw new Error(`Unknown function: ${fnName}`);
    }
  }

  /**
   * Handle SRS zome functions
   */
  private handleSrsZome(fnName: string, payload: any): any {
    switch (fnName) {
      case 'create_deck':
        return this.mockActionHash();
      case 'get_my_decks':
        return [
          this.wrapAsRecord({
            name: 'Spanish Vocabulary',
            description: 'Core 1000 Spanish words',
            card_count: 250,
            created_at: Date.now() / 1000,
          }),
          this.wrapAsRecord({
            name: 'Rust Syntax',
            description: 'Essential Rust patterns',
            card_count: 150,
            created_at: Date.now() / 1000,
          }),
        ];
      case 'get_due_cards':
        return [
          this.wrapAsRecord({
            front: '¿Cómo estás?',
            back: 'How are you?',
            status: 'Review',
            ease_factor_permille: 2500,
            interval_days: 5,
          }),
        ];
      case 'create_card':
        return this.mockActionHash();
      case 'submit_review':
        return this.wrapAsRecord(payload);
      case 'start_session':
        return this.wrapAsRecord({ session_id: `session-${Date.now()}` });
      case 'end_session':
        return this.wrapAsRecord({ cards_reviewed: 10, accuracy_permille: 850 });
      case 'get_stats':
        return [];
      default:
        throw new Error(`Unknown SRS function: ${fnName}`);
    }
  }

  /**
   * Handle Gamification zome functions
   */
  private handleGamificationZome(fnName: string, _payload: any): any {
    switch (fnName) {
      case 'get_or_create_xp':
        return this.wrapAsRecord({
          total_xp: 12500n,
          level: 15,
          xp_to_next_level: 1500,
          lifetime_xp: 12500n,
          multiplier_permille: 1000,
          last_activity_at: Date.now() / 1000,
          created_at: Date.now() / 1000 - 86400 * 30,
        });
      case 'award_xp':
        return this.mockActionHash();
      case 'get_or_create_streak':
        return this.wrapAsRecord({
          current_streak: 7,
          longest_streak: 21,
          last_activity_date: parseInt(new Date().toISOString().slice(0, 10).replace(/-/g, '')),
          freeze_available: true,
          freeze_count_used: 1,
        });
      case 'record_activity':
        return this.wrapAsRecord({ current_streak: 8 });
      case 'use_streak_freeze':
        return this.wrapAsRecord({ freeze_available: false });
      case 'get_all_badges':
        return [
          this.wrapAsRecord({
            badge_id: 'first-lesson',
            name: 'First Steps',
            description: 'Complete your first lesson',
            rarity: 'Common',
            category: 'Learning',
            xp_reward: 100,
          }),
          this.wrapAsRecord({
            badge_id: 'week-streak',
            name: 'Week Warrior',
            description: 'Maintain a 7-day streak',
            rarity: 'Rare',
            category: 'Streak',
            xp_reward: 500,
          }),
        ];
      case 'get_my_badges':
        return [
          this.wrapAsRecord({
            badge_id: 'first-lesson',
            earned_at: Date.now() / 1000 - 86400 * 7,
          }),
        ];
      case 'award_badge':
        return this.mockActionHash();
      case 'get_leaderboard':
        return {
          leaderboard_id: 'global-weekly',
          name: 'Weekly Champions',
          leaderboard_type: 'WeeklyXp',
          period: 'Weekly',
          scope: 'Global',
          entries: [
            { rank: 1, score: 5200n, display_name: 'Alice', change: 0 },
            { rank: 2, score: 4800n, display_name: 'Bob', change: 2 },
            { rank: 3, score: 4500n, display_name: 'You', change: -1 },
          ],
          updated_at: Date.now() / 1000,
        };
      default:
        throw new Error(`Unknown Gamification function: ${fnName}`);
    }
  }

  /**
   * Handle Adaptive zome functions
   */
  private handleAdaptiveZome(fnName: string, _payload: any): any {
    switch (fnName) {
      case 'get_or_create_profile':
        return this.wrapAsRecord({
          primary_style: 'Visual',
          visual_score_permille: 450,
          auditory_score_permille: 200,
          reading_score_permille: 250,
          kinesthetic_score_permille: 100,
          preferred_content_types: ['Video', 'Interactive'],
          session_duration_preference_minutes: 25,
          daily_goal_minutes: 60,
          difficulty_preference_permille: 600,
          assessment_count: 3,
          created_at: Date.now() / 1000 - 86400 * 30,
          updated_at: Date.now() / 1000,
        });
      case 'get_my_skills':
        return [
          this.wrapAsRecord({
            skill_name: 'Spanish Vocabulary',
            mastery_permille: 650,
            level: 'Competent',
            practice_count: 150,
            correct_count: 120,
          }),
          this.wrapAsRecord({
            skill_name: 'Rust Ownership',
            mastery_permille: 450,
            level: 'Beginner',
            practice_count: 45,
            correct_count: 28,
          }),
        ];
      case 'generate_recommendations':
        return [
          this.wrapAsRecord({
            recommendation_type: 'Practice',
            content_type: 'Quiz',
            reason: 'InZpd',
            confidence_permille: 850,
            priority: 1,
            estimated_duration_minutes: 10,
          }),
        ];
      case 'get_my_goals':
        return [
          this.wrapAsRecord({
            goal_id: 'daily-practice',
            title: 'Daily Practice',
            goal_type: 'DailyPractice',
            target_value: 30,
            current_value: 15,
            priority: 'High',
            is_active: true,
            is_completed: false,
          }),
        ];
      case 'get_learner_context':
        return {
          profile: { primary_style: 'Visual' },
          masteries: [],
          goals: [],
          due_for_review: [],
          strengths: [],
          weaknesses: [],
          stats: {
            total_skills: 10,
            mastered_count: 2,
            in_progress_count: 5,
            novice_count: 3,
            avg_mastery_permille: 450,
            total_attempts: 500,
            overall_accuracy_permille: 720,
            dominant_style: 'Visual',
          },
        };
      case 'analyze_flow_state':
        return {
          state: 'Flow',
          confidence_permille: 800,
          metrics: {
            skill_level_permille: 600,
            challenge_level_permille: 550,
            balance_permille: 500,
            engagement_permille: 850,
            fatigue_indicator_permille: 200,
          },
          adjustments: [],
          trend: 'Stable',
        };
      case 'get_optimal_learning_window':
        return {
          recommended_duration_minutes: 25,
          optimal_difficulty_range: [400, 700],
          best_hours: [9, 10, 14, 15],
          suggested_content_types: ['Video', 'Interactive'],
          estimated_energy_permille: 750,
          confidence_permille: 800,
        };
      case 'start_session':
        return this.wrapAsRecord({ session_id: `session-${Date.now()}` });
      case 'end_session':
        return this.wrapAsRecord({ duration_seconds: 1800, xp_earned: 250 });
      case 'predict_skill_retention':
        return {
          current_retention_permille: 850,
          retention_1h_permille: 820,
          retention_1d_permille: 680,
          retention_7d_permille: 450,
          retention_30d_permille: 280,
          stability_minutes: 2880,
          time_to_80_percent_minutes: 120,
          time_to_50_percent_minutes: 4320,
          difficulty_factor_permille: 2500,
          successful_reviews: 5,
          retrievability_permille: 800,
        };
      default:
        // Return empty for unimplemented functions
        return null;
    }
  }

  /**
   * Handle Integration zome functions
   */
  private handleIntegrationZome(fnName: string, _payload: any): any {
    switch (fnName) {
      case 'get_unified_dashboard':
        return {
          learner_profile: { primary_style: 'Visual' },
          xp: { total_xp: 12500, level: 15 },
          streak: { current_streak: 7 },
          recent_badges: [],
          recommendations: [],
          goals_in_progress: [],
          skills_due_review: [],
        };
      case 'record_learning_event':
        return this.mockActionHash();
      case 'plan_smart_session':
        return {
          session_type: 'Balanced',
          planned_duration_minutes: 25,
          activities: [
            { type: 'Review', skill: 'Spanish Vocabulary', duration: 10 },
            { type: 'NewLearning', skill: 'Grammar', duration: 10 },
            { type: 'Practice', skill: 'Listening', duration: 5 },
          ],
          break_at_minutes: 12,
        };
      case 'complete_activity':
        return {
          xp_earned: 50,
          mastery_change: 25,
          badges_earned: [],
          streak_maintained: true,
        };
      case 'get_learning_journey':
        return {
          total_days: 30,
          sessions_completed: 45,
          skills_mastered: 3,
          total_xp: 12500,
          current_goals: 2,
          completed_goals: 5,
        };
      default:
        throw new Error(`Unknown Integration function: ${fnName}`);
    }
  }

  /**
   * Simulate network latency
   */
  private async simulateLatency(): Promise<void> {
    const latency = this.config.minLatency! +
      Math.random() * (this.config.maxLatency! - this.config.minLatency!);
    await new Promise(resolve => setTimeout(resolve, latency));
  }

  /**
   * Generate a mock course
   */
  private generateMockCourse(id: string, title: string): any {
    return {
      course_id: id,
      title,
      description: `Learn ${title} from scratch`,
      instructor: 'Prof. Smith',
      syllabus: {
        modules: [
          { module_id: '1', title: 'Introduction', duration_hours: 2 },
          { module_id: '2', title: 'Core Concepts', duration_hours: 4 },
          { module_id: '3', title: 'Advanced Topics', duration_hours: 6 },
        ],
      },
      tags: ['programming', 'intermediate'],
      difficulty: 'intermediate',
      enrollment_count: Math.floor(Math.random() * 500),
      created_at: new Date().toISOString(),
    };
  }

  /**
   * Generate a mock FL round
   */
  private generateMockFlRound(id: string, state: string, participants: number): any {
    return {
      round_id: id,
      model_id: `model-${id}`,
      state,
      current_participants: participants,
      min_participants: 10,
      max_participants: 100,
      aggregation_method: 'trimmed_mean',
      clip_norm: 1.0,
      privacy_params: {
        epsilon: null,
        delta: null,
        clip_norm: 1.0,
      },
      created_at: new Date().toISOString(),
    };
  }

  /**
   * Generate a mock credential
   */
  private generateMockCredential(courseId: string): any {
    return {
      '@context': [
        'https://www.w3.org/2018/credentials/v1',
        'https://mycelix.network/credentials/v1',
      ],
      type: ['VerifiableCredential', 'EduAchievementCredential'],
      issuer: 'did:key:z6Mk...',
      issuanceDate: new Date().toISOString(),
      credentialSubject: {
        id: 'did:key:z6Mkr...',
        courseId,
        score: 80 + Math.random() * 20,
        scoreBand: 'A',
        skills: ['Critical Thinking', 'Problem Solving'],
        completionDate: new Date().toISOString().split('T')[0],
      },
      proof: {
        type: 'Ed25519Signature2020',
        created: new Date().toISOString(),
        verificationMethod: 'did:key:z6Mk...#keys-1',
        proofPurpose: 'assertionMethod',
        proofValue: 'z' + this.generateHash().slice(0, 86),
      },
    };
  }

  /**
   * Generate a random hash
   */
  private generateHash(): string {
    return Array.from({ length: 64 }, () =>
      Math.floor(Math.random() * 16).toString(16)
    ).join('');
  }
}

// Singleton instance for convenience
let mockClientInstance: MockHolochainClient | null = null;

export function getMockClient(config?: Partial<MockHolochainClientConfig>): MockHolochainClient {
  if (!mockClientInstance) {
    mockClientInstance = new MockHolochainClient(config);
  }
  return mockClientInstance;
}

export function resetMockClient(): void {
  if (mockClientInstance) {
    mockClientInstance.disconnect();
  }
  mockClientInstance = null;
}
