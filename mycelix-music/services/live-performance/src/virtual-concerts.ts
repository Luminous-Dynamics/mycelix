// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Live Performance Platform
 *
 * Virtual concerts, real-time jam sessions, DJ tools,
 * and intelligent setlist management.
 */

import { EventEmitter } from 'events';
import { WebSocket, WebSocketServer } from 'ws';

// ============================================================================
// Types - Virtual Concerts
// ============================================================================

export interface VirtualConcert {
  id: string;
  artistId: string;
  title: string;
  description: string;
  venue: VirtualVenue;
  scheduledStart: Date;
  scheduledEnd: Date;
  actualStart?: Date;
  actualEnd?: Date;
  status: ConcertStatus;
  ticketTiers: TicketTier[];
  maxCapacity: number;
  currentAttendees: number;
  setlist: SetlistItem[];
  features: ConcertFeatures;
  stats: ConcertStats;
}

export type ConcertStatus = 'scheduled' | 'live' | 'intermission' | 'ended' | 'cancelled';

export interface VirtualVenue {
  id: string;
  name: string;
  template: VenueTemplate;
  capacity: number;
  layout: VenueLayout;
  atmosphere: AtmosphereSettings;
  customizations: VenueCustomization[];
}

export type VenueTemplate =
  | 'intimate_club'
  | 'concert_hall'
  | 'stadium'
  | 'outdoor_festival'
  | 'rooftop'
  | 'underwater'
  | 'space_station'
  | 'forest_clearing'
  | 'custom';

export interface VenueLayout {
  stagePosition: { x: number; y: number; z: number };
  audienceZones: AudienceZone[];
  cameraPositions: CameraPosition[];
  lightingRigs: LightingRig[];
  screenPositions: ScreenPosition[];
}

export interface AudienceZone {
  id: string;
  name: string;
  capacity: number;
  position: { x: number; y: number; z: number };
  vipOnly: boolean;
  features: string[];
}

export interface CameraPosition {
  id: string;
  name: string;
  position: { x: number; y: number; z: number };
  rotation: { pitch: number; yaw: number; roll: number };
  fov: number;
  isDefault: boolean;
}

export interface LightingRig {
  id: string;
  type: 'spot' | 'flood' | 'laser' | 'led_panel' | 'strobe' | 'moving_head';
  position: { x: number; y: number; z: number };
  color: string;
  intensity: number;
  dmxChannel?: number;
}

export interface ScreenPosition {
  id: string;
  position: { x: number; y: number; z: number };
  size: { width: number; height: number };
  content: 'artist_feed' | 'visuals' | 'lyrics' | 'audience' | 'custom';
}

export interface AtmosphereSettings {
  fog: { enabled: boolean; density: number; color: string };
  particles: { type: string; density: number };
  ambientLight: { color: string; intensity: number };
  skybox: string;
  weather?: 'clear' | 'rain' | 'snow' | 'aurora';
}

export interface VenueCustomization {
  type: 'banner' | 'logo' | 'decoration' | 'hologram';
  asset: string;
  position: { x: number; y: number; z: number };
  scale: number;
}

export interface TicketTier {
  id: string;
  name: string;
  price: number;
  currency: string;
  maxQuantity: number;
  soldCount: number;
  perks: string[];
  zoneAccess: string[];
}

export interface SetlistItem {
  position: number;
  trackId: string;
  title: string;
  duration: number;
  notes?: string;
  visualsPreset?: string;
  lightingPreset?: string;
  status: 'pending' | 'playing' | 'played' | 'skipped';
  startedAt?: Date;
  endedAt?: Date;
}

export interface ConcertFeatures {
  chat: boolean;
  reactions: boolean;
  tipping: boolean;
  virtualMerch: boolean;
  meetAndGreet: boolean;
  backstageAccess: boolean;
  recordingAvailable: boolean;
  multiCamera: boolean;
  spatialAudio: boolean;
  vrSupport: boolean;
}

export interface ConcertStats {
  peakViewers: number;
  totalViews: number;
  chatMessages: number;
  reactions: ReactionStats;
  revenue: {
    tickets: number;
    tips: number;
    merch: number;
  };
  avgWatchTime: number;
  geographicDistribution: { country: string; count: number }[];
}

export interface ReactionStats {
  fire: number;
  heart: number;
  clap: number;
  wow: number;
  lighter: number;
  dance: number;
}

export interface Attendee {
  oderId: string visitorId
  oderId visitorId
  oderId visitorId visitorId
  oderId
  oderId visitorId
  userId: string;
  oderId visitorId
  oderId avatar  avatar: AvatarConfig;
  userId oderId
  oderId zone: string;
  userId position: { x: number; y: number; z: number };
  userId ticketTier: string;
  userId joinedAt: Date;
  userId oderId
  oderId visitorId oderId
  visitorId oderId
  oderId avatar oderId
  oderId visitorId oderId
  visitorId reactions: number;
  userId chatMessages: number;
  oderId avatar
  avatar avatar oderId
  oderId avatar
}

export interface AvatarConfig {
  oderId modelId: string;
  userId
  oderId customizations: Record<string, any>;
  oderId animations: string[];
}

// ============================================================================
// Types - Jam Sessions
// ============================================================================

export interface JamSession {
  id: string;
  hostId: string;
  name: string;
  isPublic: boolean;
  genre?: string;
  tempo: number;
  key: string;
  participants: JamParticipant[];
  maxParticipants: number;
  status: 'waiting' | 'active' | 'paused' | 'ended';
  settings: JamSettings;
  recording?: JamRecording;
  createdAt: Date;
}

export interface JamParticipant {
  userId: string;
  username: string;
  instrument: string;
  inputDevice: string;
  latency: number;
  isMuted: boolean;
  volume: number;
  pan: number;
  effects: AudioEffect[];
  isReady: boolean;
  joinedAt: Date;
}

export interface JamSettings {
  latencyCompensation: boolean;
  metronomeEnabled: boolean;
  metronomeVolume: number;
  quantizeInput: boolean;
  quantizeResolution: '1/4' | '1/8' | '1/16';
  loopEnabled: boolean;
  loopBars: number;
  recordingEnabled: boolean;
}

export interface AudioEffect {
  type: 'reverb' | 'delay' | 'distortion' | 'chorus' | 'compression' | 'eq';
  enabled: boolean;
  parameters: Record<string, number>;
}

export interface JamRecording {
  id: string;
  startedAt: Date;
  endedAt?: Date;
  tracks: RecordedTrack[];
  mixdownUrl?: string;
}

export interface RecordedTrack {
  participantId: string visitorId;
  instrument: string;
  audioUrl: string;
  duration: number;
}

// ============================================================================
// Types - DJ Tools
// ============================================================================

export interface DJSession {
  id: string;
  oderId djId: string;
  oderId name: string;
  oderId status: 'preparing' | 'live' | 'ended';
  oderId decks: DJDeck[];
  userId mixer: DJMixer;
  userId effects: DJEffectsRack;
  oderId sampler: DJSampler;
  oderId setlist: DJSetItem[];
  userId listeners: number;
  oderId recording?: string;
  oderId createdAt: Date;
}

export interface DJDeck {
  id: 'A' | 'B' | 'C' | 'D';
  loadedTrack: LoadedTrack | null;
  playbackState: 'stopped' | 'playing' | 'paused' | 'cueing';
  position: number;
  tempo: number;
  originalTempo: number;
  tempoRange: number;
  pitch: number;
  keyLock: boolean;
  currentKey: string;
  waveform: WaveformData;
  cuePoints: CuePoint[];
  loops: LoopPoint[];
  sync: boolean;
  master: boolean;
}

export interface LoadedTrack {
  id: string;
  title: string;
  artist: string;
  duration: number;
  bpm: number;
  key: string;
  waveformUrl: string;
  audioUrl: string;
  beatgrid: BeatgridData;
}

export interface WaveformData {
  peaks: number[];
  frequencies: number[][];
  resolution: number;
}

export interface BeatgridData {
  firstBeat: number;
  beatInterval: number;
  beatsPerBar: number;
  downbeats: number[];
}

export interface CuePoint {
  id: string;
  position: number;
  color: string;
  label?: string;
  type: 'cue' | 'loop_in' | 'loop_out' | 'hot_cue';
}

export interface LoopPoint {
  id: string;
  inPoint: number;
  outPoint: number;
  active: boolean;
  length: number; // in beats
}

export interface DJMixer {
  crossfader: number; // -1 to 1
  crossfaderCurve: 'sharp' | 'smooth' | 'constant';
  channels: MixerChannel[];
  masterVolume: number;
  boothVolume: number;
  headphoneVolume: number;
  headphoneMix: number; // cue/master balance
  headphoneSplit: boolean;
}

export interface MixerChannel {
  deck: 'A' | 'B' | 'C' | 'D';
  volume: number;
  eq: { high: number; mid: number; low: number };
  filter: { enabled: boolean; frequency: number; resonance: number };
  gain: number;
  cue: boolean;
  mute: boolean;
}

export interface DJEffectsRack {
  slots: EffectSlot[];
  bpmSync: boolean;
}

export interface EffectSlot {
  id: number;
  effect: DJEffect | null;
  wet: number;
  enabled: boolean;
  assignedDecks: ('A' | 'B' | 'C' | 'D')[];
}

export interface DJEffect {
  type: 'echo' | 'reverb' | 'flanger' | 'phaser' | 'filter' | 'bitcrusher' | 'gate' | 'roll';
  parameters: Record<string, number>;
  beatSync: boolean;
  beatDivision: number;
}

export interface DJSampler {
  pads: SamplerPad[];
  volume: number;
  outputChannel: 'master' | 'A' | 'B';
}

export interface SamplerPad {
  id: number;
  sample: { id: string; name: string; url: string; duration: number } | null;
  playMode: 'oneshot' | 'hold' | 'loop';
  volume: number;
  isPlaying: boolean;
}

export interface DJSetItem {
  position: number;
  trackId: string;
  title: string;
  artist: string;
  bpm: number;
  key: string;
  playedAt?: Date;
  transitionType?: 'cut' | 'fade' | 'mix' | 'slam';
  transitionDuration?: number;
}

// ============================================================================
// Types - Setlist Builder
// ============================================================================

export interface SmartSetlist {
  id: string;
  name: string;
  artistId: string;
  tracks: SmartSetlistTrack[];
  totalDuration: number;
  constraints: SetlistConstraints;
  suggestions: SetlistSuggestion[];
  energyCurve: number[];
  createdAt: Date;
  updatedAt: Date;
}

export interface SmartSetlistTrack {
  position: number;
  trackId: string;
  title: string;
  duration: number;
  bpm: number;
  key: string;
  energy: number;
  transitionScore: number;
  crowdResponse?: number;
  notes?: string;
}

export interface SetlistConstraints {
  targetDuration: number;
  maxDuration: number;
  minDuration: number;
  energyProfile: 'building' | 'peaks_valleys' | 'steady' | 'descending' | 'custom';
  mustInclude: string[];
  mustExclude: string[];
  encoreEnabled: boolean;
  encoreTracks: number;
}

export interface SetlistSuggestion {
  type: 'add' | 'remove' | 'reorder' | 'transition';
  position?: number;
  trackId?: string;
  reason: string;
  impact: {
    energy: number;
    flow: number;
    crowdResponse: number;
  };
}

// ============================================================================
// Virtual Concert Service
// ============================================================================

class VirtualConcertService extends EventEmitter {
  private concerts: Map<string, VirtualConcert> = new Map();
  private attendees: Map<string, Map<string, Attendee>> = new Map();
  private wsConnections: Map<string, Set<WebSocket>> = new Map();

  async createConcert(
    artistId: string,
    data: {
      title: string;
      description: string;
      venue: Partial<VirtualVenue>;
      scheduledStart: Date;
      scheduledEnd: Date;
      ticketTiers: Omit<TicketTier, 'id' | 'soldCount'>[];
      features: Partial<ConcertFeatures>;
    }
  ): Promise<VirtualConcert> {
    const concertId = `concert_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    const venue = this.createVenue(data.venue);

    const concert: VirtualConcert = {
      id: concertId,
      artistId,
      title: data.title,
      description: data.description,
      venue,
      scheduledStart: data.scheduledStart,
      scheduledEnd: data.scheduledEnd,
      status: 'scheduled',
      ticketTiers: data.ticketTiers.map((tier, i) => ({
        ...tier,
        id: `tier_${i}`,
        soldCount: 0,
      })),
      maxCapacity: venue.capacity,
      currentAttendees: 0,
      setlist: [],
      features: {
        chat: true,
        reactions: true,
        tipping: true,
        virtualMerch: false,
        meetAndGreet: false,
        backstageAccess: false,
        recordingAvailable: true,
        multiCamera: true,
        spatialAudio: true,
        vrSupport: false,
        ...data.features,
      },
      stats: {
        peakViewers: 0,
        totalViews: 0,
        chatMessages: 0,
        reactions: { fire: 0, heart: 0, clap: 0, wow: 0, lighter: 0, dance: 0 },
        revenue: { tickets: 0, tips: 0, merch: 0 },
        avgWatchTime: 0,
        geographicDistribution: [],
      },
    };

    this.concerts.set(concertId, concert);
    this.attendees.set(concertId, new Map());
    this.wsConnections.set(concertId, new Set());

    return concert;
  }

  private createVenue(data: Partial<VirtualVenue>): VirtualVenue {
    const template = data.template || 'concert_hall';
    const baseVenue = this.getVenueTemplate(template);

    return {
      ...baseVenue,
      ...data,
      id: `venue_${Date.now()}`,
    };
  }

  private getVenueTemplate(template: VenueTemplate): VirtualVenue {
    const templates: Record<VenueTemplate, Partial<VirtualVenue>> = {
      intimate_club: {
        name: 'Intimate Club',
        capacity: 500,
        layout: {
          stagePosition: { x: 0, y: 2, z: 0 },
          audienceZones: [
            { id: 'floor', name: 'Floor', capacity: 400, position: { x: 0, y: 0, z: 10 }, vipOnly: false, features: [] },
            { id: 'vip', name: 'VIP Booth', capacity: 100, position: { x: -10, y: 1, z: 5 }, vipOnly: true, features: ['private_chat'] },
          ],
          cameraPositions: [
            { id: 'front', name: 'Front', position: { x: 0, y: 2, z: 15 }, rotation: { pitch: 0, yaw: 0, roll: 0 }, fov: 60, isDefault: true },
          ],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: true, density: 0.3, color: '#1a1a2e' },
          particles: { type: 'dust', density: 0.2 },
          ambientLight: { color: '#2d2d44', intensity: 0.3 },
          skybox: 'club_interior',
        },
      },
      stadium: {
        name: 'Stadium',
        capacity: 50000,
        layout: {
          stagePosition: { x: 0, y: 5, z: 0 },
          audienceZones: [
            { id: 'pit', name: 'Pit', capacity: 5000, position: { x: 0, y: 0, z: 20 }, vipOnly: false, features: [] },
            { id: 'floor', name: 'Floor', capacity: 20000, position: { x: 0, y: 0, z: 50 }, vipOnly: false, features: [] },
            { id: 'stands', name: 'Stands', capacity: 25000, position: { x: 0, y: 10, z: 80 }, vipOnly: false, features: [] },
          ],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: false, density: 0, color: '#000000' },
          particles: { type: 'confetti', density: 0.1 },
          ambientLight: { color: '#1a1a1a', intensity: 0.2 },
          skybox: 'night_sky',
        },
      },
      concert_hall: {
        name: 'Concert Hall',
        capacity: 5000,
        layout: {
          stagePosition: { x: 0, y: 3, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: true, density: 0.2, color: '#1a1a2e' },
          particles: { type: 'none', density: 0 },
          ambientLight: { color: '#2a2a3a', intensity: 0.4 },
          skybox: 'theater_interior',
        },
      },
      outdoor_festival: {
        name: 'Outdoor Festival',
        capacity: 100000,
        layout: {
          stagePosition: { x: 0, y: 8, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: false, density: 0, color: '#000000' },
          particles: { type: 'fireflies', density: 0.3 },
          ambientLight: { color: '#ffeaa7', intensity: 0.6 },
          skybox: 'sunset',
        },
      },
      rooftop: {
        name: 'Rooftop',
        capacity: 1000,
        layout: {
          stagePosition: { x: 0, y: 1, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: false, density: 0, color: '#000000' },
          particles: { type: 'none', density: 0 },
          ambientLight: { color: '#ff7675', intensity: 0.5 },
          skybox: 'city_night',
        },
      },
      underwater: {
        name: 'Underwater Dome',
        capacity: 2000,
        layout: {
          stagePosition: { x: 0, y: 2, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: true, density: 0.4, color: '#0984e3' },
          particles: { type: 'bubbles', density: 0.5 },
          ambientLight: { color: '#74b9ff', intensity: 0.4 },
          skybox: 'ocean_floor',
        },
      },
      space_station: {
        name: 'Space Station',
        capacity: 3000,
        layout: {
          stagePosition: { x: 0, y: 0, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: false, density: 0, color: '#000000' },
          particles: { type: 'stars', density: 0.8 },
          ambientLight: { color: '#2d3436', intensity: 0.3 },
          skybox: 'deep_space',
        },
      },
      forest_clearing: {
        name: 'Forest Clearing',
        capacity: 5000,
        layout: {
          stagePosition: { x: 0, y: 2, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: true, density: 0.3, color: '#2d5016' },
          particles: { type: 'fireflies', density: 0.6 },
          ambientLight: { color: '#00b894', intensity: 0.4 },
          skybox: 'forest_night',
        },
      },
      custom: {
        name: 'Custom Venue',
        capacity: 10000,
        layout: {
          stagePosition: { x: 0, y: 2, z: 0 },
          audienceZones: [],
          cameraPositions: [],
          lightingRigs: [],
          screenPositions: [],
        },
        atmosphere: {
          fog: { enabled: false, density: 0, color: '#000000' },
          particles: { type: 'none', density: 0 },
          ambientLight: { color: '#ffffff', intensity: 0.5 },
          skybox: 'default',
        },
      },
    };

    return {
      id: '',
      template,
      customizations: [],
      ...templates[template],
    } as VirtualVenue;
  }

  async startConcert(concertId: string): Promise<void> {
    const concert = this.concerts.get(concertId);
    if (!concert) throw new Error('Concert not found');

    concert.status = 'live';
    concert.actualStart = new Date();

    this.broadcastToAttendees(concertId, {
      type: 'concert_started',
      timestamp: concert.actualStart,
    });

    this.emit('concert_started', concert);
  }

  async endConcert(concertId: string): Promise<void> {
    const concert = this.concerts.get(concertId);
    if (!concert) throw new Error('Concert not found');

    concert.status = 'ended';
    concert.actualEnd = new Date();

    this.broadcastToAttendees(concertId, {
      type: 'concert_ended',
      timestamp: concert.actualEnd,
      stats: concert.stats,
    });

    this.emit('concert_ended', concert);
  }

  async joinConcert(
    concertId: string,
    userId: string,
    ticketId: string,
    avatar: AvatarConfig
  ): Promise<{ attendee: Attendee; streamUrl: string }> {
    const concert = this.concerts.get(concertId);
    if (!concert) throw new Error('Concert not found');

    const attendeeMap = this.attendees.get(concertId)!;

    const attendee: Attendee = {
      oderId visitorId
      oderId visitorId visitorId
      userId,
      oderId visitorId
      avatar,
      zone: 'floor',
      position: this.getRandomPosition(concert.venue.layout.audienceZones[0]}),
      ticketTier: ticketId,
      joinedAt: new Date(),
      reactions: 0,
      chatMessages: 0,
    };

    attendeeMap.set(userId, attendee);
    concert.currentAttendees = attendeeMap.size;
    concert.stats.totalViews++;
    concert.stats.peakViewers = Math.max(concert.stats.peakViewers, attendeeMap.size);

    return {
      attendee,
      streamUrl: `wss://stream.mycelix.io/concert/${concertId}`,
    };
  }

  async sendReaction(concertId: string, oderId visitorId
oderId: string, reaction: keyof ReactionStats): Promise<void> {
    const concert = this.concerts.get(concertId);
    if (!concert) return;

    concert.stats.reactions[reaction]++;

    this.broadcastToAttendees(concertId, {
      type: 'reaction',
      oderId visitorId
userId,
      reaction,
      timestamp: new Date(),
    });
  }

  private broadcastToAttendees(concertId: string, message: any): void {
    const connections = this.wsConnections.get(concertId);
    if (!connections) return;

    const data = JSON.stringify(message);
    for (const ws of connections) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(data);
      }
    }
  }

  private getRandomPosition(zone: AudienceZone): { x: number; y: number; z: number } {
    return {
      x: zone.position.x + (Math.random() - 0.5) * 20,
      y: zone.position.y,
      z: zone.position.z + (Math.random() - 0.5) * 20,
    };
  }

  getConcert(concertId: string): VirtualConcert | undefined {
    return this.concerts.get(concertId);
  }
}

// ============================================================================
// Jam Session Service
// ============================================================================

class JamSessionService extends EventEmitter {
  private sessions: Map<string, JamSession> = new Map();
  private audioStreams: Map<string, Map<string, any>> = new Map();

  async createSession(
    hostId: string,
    data: {
      name: string;
      isPublic: boolean;
      genre?: string;
      tempo: number;
      key: string;
      maxParticipants: number;
    }
  ): Promise<JamSession> {
    const sessionId = `jam_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    const session: JamSession = {
      id: sessionId,
      hostId,
      name: data.name,
      isPublic: data.isPublic,
      genre: data.genre,
      tempo: data.tempo,
      key: data.key,
      participants: [],
      maxParticipants: data.maxParticipants,
      status: 'waiting',
      settings: {
        latencyCompensation: true,
        metronomeEnabled: true,
        metronomeVolume: 0.5,
        quantizeInput: false,
        quantizeResolution: '1/8',
        loopEnabled: false,
        loopBars: 4,
        recordingEnabled: true,
      },
      createdAt: new Date(),
    };

    this.sessions.set(sessionId, session);
    this.audioStreams.set(sessionId, new Map());

    return session;
  }

  async joinSession(
    sessionId: string,
    userId: string,
    data: { username: string; instrument: string; inputDevice: string }
  ): Promise<JamParticipant> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    if (session.participants.length >= session.maxParticipants) {
      throw new Error('Session is full');
    }

    const participant: JamParticipant = {
      oderId visitorId
userId,
      username: data.username,
      instrument: data.instrument,
      inputDevice: data.inputDevice,
      latency: 0,
      isMuted: false,
      volume: 1,
      pan: 0,
      effects: [],
      isReady: false,
      joinedAt: new Date(),
    };

    session.participants.push(participant);

    this.emit('participant_joined', { sessionId, participant });

    return participant;
  }

  async measureLatency(sessionId: string, userId: string): Promise<number> {
    // Would measure round-trip audio latency
    const latency = 20 + Math.random() * 30; // Simulated 20-50ms

    const session = this.sessions.get(sessionId);
    if (session) {
      const participant = session.participants.find(p => p.userId === oderId visitorId
userId);
      if (participant) {
        participant.latency = latency;
      }
    }

    return latency;
  }

  async startSession(sessionId: string): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    // Ensure all participants are ready
    const notReady = session.participants.filter(p => !p.isReady);
    if (notReady.length > 0) {
      throw new Error(`${notReady.length} participants not ready`);
    }

    session.status = 'active';

    if (session.settings.recordingEnabled) {
      session.recording = {
        id: `rec_${Date.now()}`,
        startedAt: new Date(),
        tracks: [],
      };
    }

    this.emit('session_started', session);
  }

  async setParticipantEffect(
    sessionId: string,
    oderId visitorId
userId: string,
    effect: AudioEffect
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.find(p => p.userId === oderId visitorId
userId);
    if (!participant) return;

    const existingIndex = participant.effects.findIndex(e => e.type === effect.type);
    if (existingIndex >= 0) {
      participant.effects[existingIndex] = effect;
    } else {
      participant.effects.push(effect);
    }

    this.emit('effect_changed', { sessionId, oderId visitorId
userId, effect });
  }

  async endSession(sessionId: string): Promise<JamRecording | undefined> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    session.status = 'ended';

    if (session.recording) {
      session.recording.endedAt = new Date();

      // Would create mixdown of all tracks
      session.recording.mixdownUrl = await this.createMixdown(session.recording);
    }

    this.emit('session_ended', session);

    return session.recording;
  }

  private async createMixdown(recording: JamRecording): Promise<string> {
    // Would mix all tracks together with latency compensation
    return `/recordings/mixdown_${recording.id}.wav`;
  }

  getPublicSessions(): JamSession[] {
    return Array.from(this.sessions.values()).filter(
      s => s.isPublic && s.status !== 'ended'
    );
  }
}

// ============================================================================
// DJ Service
// ============================================================================

class DJService extends EventEmitter {
  private sessions: Map<string, DJSession> = new Map();

  async createSession(djId: string, name: string): Promise<DJSession> {
    const sessionId = `dj_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    const session: DJSession = {
      id: sessionId,
      djId,
      name,
      status: 'preparing',
      decks: [
        this.createEmptyDeck('A'),
        this.createEmptyDeck('B'),
      ],
      mixer: {
        crossfader: 0,
        crossfaderCurve: 'smooth',
        channels: [
          { deck: 'A', volume: 1, eq: { high: 0, mid: 0, low: 0 }, filter: { enabled: false, frequency: 1000, resonance: 0.5 }, gain: 0, cue: false, mute: false },
          { deck: 'B', volume: 1, eq: { high: 0, mid: 0, low: 0 }, filter: { enabled: false, frequency: 1000, resonance: 0.5 }, gain: 0, cue: false, mute: false },
        ],
        masterVolume: 1,
        boothVolume: 0.8,
        headphoneVolume: 1,
        headphoneMix: 0.5,
        headphoneSplit: false,
      },
      effects: {
        slots: [
          { id: 1, effect: null, wet: 0, enabled: false, assignedDecks: [] },
          { id: 2, effect: null, wet: 0, enabled: false, assignedDecks: [] },
          { id: 3, effect: null, wet: 0, enabled: false, assignedDecks: [] },
        ],
        bpmSync: true,
      },
      sampler: {
        pads: Array.from({ length: 8 }, (_, i) => ({
          id: i,
          sample: null,
          playMode: 'oneshot' as const,
          volume: 1,
          isPlaying: false,
        })),
        volume: 1,
        outputChannel: 'master',
      },
      setlist: [],
      listeners: 0,
      createdAt: new Date(),
    };

    this.sessions.set(sessionId, session);

    return session;
  }

  private createEmptyDeck(id: 'A' | 'B' | 'C' | 'D'): DJDeck {
    return {
      id,
      loadedTrack: null,
      playbackState: 'stopped',
      position: 0,
      tempo: 120,
      originalTempo: 120,
      tempoRange: 8,
      pitch: 0,
      keyLock: false,
      currentKey: 'C',
      waveform: { peaks: [], frequencies: [], resolution: 0 },
      cuePoints: [],
      loops: [],
      sync: false,
      master: id === 'A',
    };
  }

  async loadTrack(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    track: LoadedTrack
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck) throw new Error('Deck not found');

    deck.loadedTrack = track;
    deck.originalTempo = track.bpm;
    deck.tempo = track.bpm;
    deck.currentKey = track.key;
    deck.position = 0;
    deck.playbackState = 'stopped';

    // Generate waveform data
    deck.waveform = await this.generateWaveform(track);

    this.emit('track_loaded', { sessionId, deckId, track });
  }

  async play(sessionId: string, deckId: 'A' | 'B' | 'C' | 'D'): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck || !deck.loadedTrack) return;

    deck.playbackState = 'playing';

    this.emit('deck_play', { sessionId, deckId });
  }

  async pause(sessionId: string, deckId: 'A' | 'B' | 'C' | 'D'): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck) return;

    deck.playbackState = 'paused';

    this.emit('deck_pause', { sessionId, deckId });
  }

  async setTempo(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    tempo: number
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck) return;

    deck.tempo = tempo;
    deck.pitch = ((tempo - deck.originalTempo) / deck.originalTempo) * 100;

    this.emit('tempo_changed', { sessionId, deckId, tempo });
  }

  async syncDeck(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    enabled: boolean
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck) return;

    deck.sync = enabled;

    if (enabled) {
      // Find master deck and sync to its tempo
      const master = session.decks.find(d => d.master && d.id !== deckId);
      if (master) {
        deck.tempo = master.tempo;
      }
    }

    this.emit('sync_changed', { sessionId, deckId, enabled });
  }

  async setCrossfader(sessionId: string, value: number): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    session.mixer.crossfader = Math.max(-1, Math.min(1, value));

    this.emit('crossfader_changed', { sessionId, value: session.mixer.crossfader });
  }

  async setEQ(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    band: 'high' | 'mid' | 'low',
    value: number
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const channel = session.mixer.channels.find(c => c.deck === deckId);
    if (!channel) return;

    channel.eq[band] = Math.max(-12, Math.min(12, value));

    this.emit('eq_changed', { sessionId, deckId, band, value: channel.eq[band] });
  }

  async applyEffect(
    sessionId: string,
    slotId: number,
    effect: DJEffect
  ): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    const slot = session.effects.slots.find(s => s.id === slotId);
    if (!slot) return;

    slot.effect = effect;
    slot.enabled = true;

    this.emit('effect_applied', { sessionId, slotId, effect });
  }

  async setCuePoint(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    position: number,
    color: string
  ): Promise<CuePoint> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck) throw new Error('Deck not found');

    const cue: CuePoint = {
      id: `cue_${Date.now()}`,
      position,
      color,
      type: 'hot_cue',
    };

    deck.cuePoints.push(cue);

    return cue;
  }

  async setLoop(
    sessionId: string,
    deckId: 'A' | 'B' | 'C' | 'D',
    beats: number
  ): Promise<LoopPoint> {
    const session = this.sessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    const deck = session.decks.find(d => d.id === deckId);
    if (!deck || !deck.loadedTrack) throw new Error('Deck not found or no track loaded');

    const beatDuration = 60 / deck.tempo;
    const loopDuration = beats * beatDuration;

    const loop: LoopPoint = {
      id: `loop_${Date.now()}`,
      inPoint: deck.position,
      outPoint: deck.position + loopDuration,
      active: true,
      length: beats,
    };

    // Deactivate other loops
    deck.loops.forEach(l => l.active = false);
    deck.loops.push(loop);

    return loop;
  }

  private async generateWaveform(track: LoadedTrack): Promise<WaveformData> {
    // Would analyze audio and generate waveform
    return {
      peaks: [],
      frequencies: [],
      resolution: 1000,
    };
  }

  getSession(sessionId: string): DJSession | undefined {
    return this.sessions.get(sessionId);
  }
}

// ============================================================================
// Setlist Builder Service
// ============================================================================

class SetlistBuilderService {
  private setlists: Map<string, SmartSetlist> = new Map();

  async createSetlist(
    artistId: string,
    name: string,
    constraints: SetlistConstraints
  ): Promise<SmartSetlist> {
    const setlistId = `setlist_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    const setlist: SmartSetlist = {
      id: setlistId,
      name,
      artistId,
      tracks: [],
      totalDuration: 0,
      constraints,
      suggestions: [],
      energyCurve: [],
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    this.setlists.set(setlistId, setlist);

    return setlist;
  }

  async addTrack(
    setlistId: string,
    track: Omit<SmartSetlistTrack, 'position' | 'transitionScore'>
  ): Promise<SmartSetlist> {
    const setlist = this.setlists.get(setlistId);
    if (!setlist) throw new Error('Setlist not found');

    const position = setlist.tracks.length;

    const smartTrack: SmartSetlistTrack = {
      ...track,
      position,
      transitionScore: position > 0
        ? this.calculateTransitionScore(setlist.tracks[position - 1], track)
        : 1,
    };

    setlist.tracks.push(smartTrack);
    setlist.totalDuration += track.duration;
    setlist.energyCurve = this.calculateEnergyCurve(setlist.tracks);
    setlist.suggestions = this.generateSuggestions(setlist);
    setlist.updatedAt = new Date();

    return setlist;
  }

  async reorderTrack(
    setlistId: string,
    fromPosition: number,
    toPosition: number
  ): Promise<SmartSetlist> {
    const setlist = this.setlists.get(setlistId);
    if (!setlist) throw new Error('Setlist not found');

    const [track] = setlist.tracks.splice(fromPosition, 1);
    setlist.tracks.splice(toPosition, 0, track);

    // Recalculate positions and transition scores
    setlist.tracks.forEach((t, i) => {
      t.position = i;
      t.transitionScore = i > 0
        ? this.calculateTransitionScore(setlist.tracks[i - 1], t)
        : 1;
    });

    setlist.energyCurve = this.calculateEnergyCurve(setlist.tracks);
    setlist.suggestions = this.generateSuggestions(setlist);
    setlist.updatedAt = new Date();

    return setlist;
  }

  async optimizeSetlist(setlistId: string): Promise<SmartSetlist> {
    const setlist = this.setlists.get(setlistId);
    if (!setlist) throw new Error('Setlist not found');

    // Sort tracks to optimize for transitions and energy flow
    const optimized = this.optimizeTrackOrder(
      setlist.tracks,
      setlist.constraints.energyProfile
    );

    setlist.tracks = optimized;
    setlist.energyCurve = this.calculateEnergyCurve(setlist.tracks);
    setlist.suggestions = this.generateSuggestions(setlist);
    setlist.updatedAt = new Date();

    return setlist;
  }

  async suggestTracks(
    setlistId: string,
    catalog: SmartSetlistTrack[]
  ): Promise<SmartSetlistTrack[]> {
    const setlist = this.setlists.get(setlistId);
    if (!setlist) throw new Error('Setlist not found');

    const remainingDuration = setlist.constraints.targetDuration - setlist.totalDuration;
    if (remainingDuration <= 0) return [];

    // Filter catalog to tracks that would fit
    const candidates = catalog.filter(t => {
      // Not already in setlist
      if (setlist.tracks.some(st => st.trackId === t.trackId)) return false;

      // Not excluded
      if (setlist.constraints.mustExclude.includes(t.trackId)) return false;

      // Fits in remaining time
      if (t.duration > remainingDuration + 60) return false;

      return true;
    });

    // Score candidates based on transition from last track
    const lastTrack = setlist.tracks[setlist.tracks.length - 1];
    const scored = candidates.map(t => ({
      track: t,
      score: lastTrack ? this.calculateTransitionScore(lastTrack, t) : 0.5,
    }));

    // Return top suggestions
    return scored
      .sort((a, b) => b.score - a.score)
      .slice(0, 10)
      .map(s => s.track);
  }

  private calculateTransitionScore(
    from: SmartSetlistTrack | Omit<SmartSetlistTrack, 'position' | 'transitionScore'>,
    to: Omit<SmartSetlistTrack, 'position' | 'transitionScore'>
  ): number {
    let score = 0;

    // BPM compatibility (closer is better)
    const bpmDiff = Math.abs(from.bpm - to.bpm);
    if (bpmDiff === 0) score += 0.3;
    else if (bpmDiff <= 3) score += 0.25;
    else if (bpmDiff <= 6) score += 0.15;
    else if (bpmDiff <= 10) score += 0.05;

    // Key compatibility (using Camelot wheel logic)
    const keyScore = this.getKeyCompatibility(from.key, to.key);
    score += keyScore * 0.3;

    // Energy flow (smooth transitions)
    const energyDiff = Math.abs(from.energy - to.energy);
    if (energyDiff <= 0.1) score += 0.2;
    else if (energyDiff <= 0.2) score += 0.15;
    else if (energyDiff <= 0.3) score += 0.1;

    // Variety bonus (not too similar)
    score += 0.2;

    return Math.min(1, score);
  }

  private getKeyCompatibility(key1: string, key2: string): number {
    // Simplified Camelot wheel compatibility
    const camelot: Record<string, number> = {
      'C': 8, 'Am': 8,
      'G': 9, 'Em': 9,
      'D': 10, 'Bm': 10,
      'A': 11, 'F#m': 11,
      'E': 12, 'C#m': 12,
      'B': 1, 'G#m': 1,
      'F#': 2, 'D#m': 2,
      'Db': 3, 'Bbm': 3,
      'Ab': 4, 'Fm': 4,
      'Eb': 5, 'Cm': 5,
      'Bb': 6, 'Gm': 6,
      'F': 7, 'Dm': 7,
    };

    const c1 = camelot[key1] || 0;
    const c2 = camelot[key2] || 0;

    if (c1 === 0 || c2 === 0) return 0.5;

    const diff = Math.abs(c1 - c2);
    const circularDiff = Math.min(diff, 12 - diff);

    if (circularDiff === 0) return 1;
    if (circularDiff === 1) return 0.9;
    if (circularDiff === 2) return 0.6;
    return 0.3;
  }

  private calculateEnergyCurve(tracks: SmartSetlistTrack[]): number[] {
    return tracks.map(t => t.energy);
  }

  private optimizeTrackOrder(
    tracks: SmartSetlistTrack[],
    profile: SetlistConstraints['energyProfile']
  ): SmartSetlistTrack[] {
    const sorted = [...tracks];

    switch (profile) {
      case 'building':
        sorted.sort((a, b) => a.energy - b.energy);
        break;
      case 'descending':
        sorted.sort((a, b) => b.energy - a.energy);
        break;
      case 'peaks_valleys':
        // Alternate high and low energy
        const high = sorted.filter(t => t.energy >= 0.5).sort((a, b) => b.energy - a.energy);
        const low = sorted.filter(t => t.energy < 0.5).sort((a, b) => a.energy - b.energy);
        const result: SmartSetlistTrack[] = [];
        const maxLen = Math.max(high.length, low.length);
        for (let i = 0; i < maxLen; i++) {
          if (high[i]) result.push(high[i]);
          if (low[i]) result.push(low[i]);
        }
        return result.map((t, i) => ({ ...t, position: i }));
      case 'steady':
        // Group by similar energy
        sorted.sort((a, b) => Math.abs(a.energy - 0.5) - Math.abs(b.energy - 0.5));
        break;
    }

    // Recalculate positions
    return sorted.map((t, i) => ({ ...t, position: i }));
  }

  private generateSuggestions(setlist: SmartSetlist): SetlistSuggestion[] {
    const suggestions: SetlistSuggestion[] = [];

    // Check for poor transitions
    for (let i = 1; i < setlist.tracks.length; i++) {
      if (setlist.tracks[i].transitionScore < 0.5) {
        suggestions.push({
          type: 'reorder',
          position: i,
          reason: `Weak transition between "${setlist.tracks[i - 1].title}" and "${setlist.tracks[i].title}"`,
          impact: { energy: 0, flow: 0.3, crowdResponse: 0.2 },
        });
      }
    }

    // Check duration constraints
    if (setlist.totalDuration < setlist.constraints.minDuration) {
      suggestions.push({
        type: 'add',
        reason: `Setlist is ${Math.round((setlist.constraints.minDuration - setlist.totalDuration) / 60)} minutes short of minimum duration`,
        impact: { energy: 0, flow: 0, crowdResponse: 0.1 },
      });
    }

    if (setlist.totalDuration > setlist.constraints.maxDuration) {
      suggestions.push({
        type: 'remove',
        reason: `Setlist is ${Math.round((setlist.totalDuration - setlist.constraints.maxDuration) / 60)} minutes over maximum duration`,
        impact: { energy: 0, flow: 0, crowdResponse: -0.1 },
      });
    }

    // Check must-include tracks
    for (const trackId of setlist.constraints.mustInclude) {
      if (!setlist.tracks.some(t => t.trackId === trackId)) {
        suggestions.push({
          type: 'add',
          trackId,
          reason: 'Required track not in setlist',
          impact: { energy: 0, flow: 0, crowdResponse: 0.3 },
        });
      }
    }

    return suggestions;
  }

  getSetlist(setlistId: string): SmartSetlist | undefined {
    return this.setlists.get(setlistId);
  }
}

// ============================================================================
// Exports
// ============================================================================

export const virtualConcerts = new VirtualConcertService();
export const jamSessions = new JamSessionService();
export const djService = new DJService();
export const setlistBuilder = new SetlistBuilderService();

export default {
  concerts: virtualConcerts,
  jams: jamSessions,
  dj: djService,
  setlists: setlistBuilder,
};
