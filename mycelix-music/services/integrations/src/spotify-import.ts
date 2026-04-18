// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Spotify Import Service
 *
 * Handles importing user data from Spotify including playlists,
 * library, listening history, and followed artists.
 */

// ============================================================================
// Types
// ============================================================================

export interface SpotifyCredentials {
  accessToken: string;
  refreshToken: string;
  expiresAt: Date;
  scope: string[];
}

export interface SpotifyPlaylist {
  id: string;
  name: string;
  description: string;
  imageUrl?: string;
  trackCount: number;
  isPublic: boolean;
  ownerId: string;
  ownerName: string;
}

export interface SpotifyTrack {
  id: string;
  name: string;
  artists: { id: string; name: string }[];
  album: {
    id: string;
    name: string;
    imageUrl?: string;
    releaseDate: string;
  };
  durationMs: number;
  isrc?: string;
  previewUrl?: string;
}

export interface SpotifyArtist {
  id: string;
  name: string;
  imageUrl?: string;
  genres: string[];
  followers: number;
}

export interface ImportProgress {
  status: 'pending' | 'in_progress' | 'completed' | 'failed';
  phase: ImportPhase;
  totalItems: number;
  processedItems: number;
  matchedItems: number;
  errors: ImportError[];
  startedAt: Date;
  completedAt?: Date;
}

export type ImportPhase =
  | 'authenticating'
  | 'fetching_playlists'
  | 'fetching_library'
  | 'fetching_artists'
  | 'fetching_history'
  | 'matching_tracks'
  | 'creating_playlists'
  | 'complete';

export interface ImportError {
  type: 'fetch' | 'match' | 'create';
  itemId?: string;
  itemName?: string;
  message: string;
}

export interface ImportResult {
  playlistsImported: number;
  tracksMatched: number;
  tracksUnmatched: number;
  artistsFollowed: number;
  likedSongsImported: number;
  historyImported: boolean;
}

export interface TrackMatch {
  spotifyTrack: SpotifyTrack;
  mycelixTrackId: string | null;
  confidence: number;
  matchMethod: 'isrc' | 'metadata' | 'audio' | 'none';
}

type ProgressCallback = (progress: ImportProgress) => void;

// ============================================================================
// Spotify Import Service
// ============================================================================

class SpotifyImportService {
  private readonly SPOTIFY_API = 'https://api.spotify.com/v1';
  private readonly CLIENT_ID = process.env.SPOTIFY_CLIENT_ID || '';
  private readonly CLIENT_SECRET = process.env.SPOTIFY_CLIENT_SECRET || '';
  private readonly REDIRECT_URI = process.env.SPOTIFY_REDIRECT_URI || '';

  private importJobs: Map<string, ImportProgress> = new Map();

  // ============================================================================
  // OAuth
  // ============================================================================

  getAuthorizationUrl(state: string): string {
    const scopes = [
      'user-library-read',
      'playlist-read-private',
      'playlist-read-collaborative',
      'user-follow-read',
      'user-top-read',
      'user-read-recently-played',
    ];

    const params = new URLSearchParams({
      client_id: this.CLIENT_ID,
      response_type: 'code',
      redirect_uri: this.REDIRECT_URI,
      scope: scopes.join(' '),
      state,
      show_dialog: 'true',
    });

    return `https://accounts.spotify.com/authorize?${params.toString()}`;
  }

  async exchangeCode(code: string): Promise<SpotifyCredentials> {
    const response = await fetch('https://accounts.spotify.com/api/token', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
        Authorization: `Basic ${Buffer.from(`${this.CLIENT_ID}:${this.CLIENT_SECRET}`).toString('base64')}`,
      },
      body: new URLSearchParams({
        grant_type: 'authorization_code',
        code,
        redirect_uri: this.REDIRECT_URI,
      }),
    });

    const data = await response.json();

    return {
      accessToken: data.access_token,
      refreshToken: data.refresh_token,
      expiresAt: new Date(Date.now() + data.expires_in * 1000),
      scope: data.scope.split(' '),
    };
  }

  async refreshToken(refreshToken: string): Promise<SpotifyCredentials> {
    const response = await fetch('https://accounts.spotify.com/api/token', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
        Authorization: `Basic ${Buffer.from(`${this.CLIENT_ID}:${this.CLIENT_SECRET}`).toString('base64')}`,
      },
      body: new URLSearchParams({
        grant_type: 'refresh_token',
        refresh_token: refreshToken,
      }),
    });

    const data = await response.json();

    return {
      accessToken: data.access_token,
      refreshToken: data.refresh_token || refreshToken,
      expiresAt: new Date(Date.now() + data.expires_in * 1000),
      scope: data.scope.split(' '),
    };
  }

  // ============================================================================
  // Data Fetching
  // ============================================================================

  async getUserPlaylists(
    credentials: SpotifyCredentials
  ): Promise<SpotifyPlaylist[]> {
    const playlists: SpotifyPlaylist[] = [];
    let offset = 0;
    const limit = 50;

    while (true) {
      const response = await this.spotifyRequest(
        credentials,
        `/me/playlists?limit=${limit}&offset=${offset}`
      );

      for (const item of response.items) {
        playlists.push({
          id: item.id,
          name: item.name,
          description: item.description || '',
          imageUrl: item.images?.[0]?.url,
          trackCount: item.tracks.total,
          isPublic: item.public,
          ownerId: item.owner.id,
          ownerName: item.owner.display_name,
        });
      }

      if (!response.next) break;
      offset += limit;
    }

    return playlists;
  }

  async getPlaylistTracks(
    credentials: SpotifyCredentials,
    playlistId: string
  ): Promise<SpotifyTrack[]> {
    const tracks: SpotifyTrack[] = [];
    let offset = 0;
    const limit = 100;

    while (true) {
      const response = await this.spotifyRequest(
        credentials,
        `/playlists/${playlistId}/tracks?limit=${limit}&offset=${offset}`
      );

      for (const item of response.items) {
        if (item.track && item.track.type === 'track') {
          tracks.push(this.parseTrack(item.track));
        }
      }

      if (!response.next) break;
      offset += limit;
    }

    return tracks;
  }

  async getLikedSongs(credentials: SpotifyCredentials): Promise<SpotifyTrack[]> {
    const tracks: SpotifyTrack[] = [];
    let offset = 0;
    const limit = 50;

    while (true) {
      const response = await this.spotifyRequest(
        credentials,
        `/me/tracks?limit=${limit}&offset=${offset}`
      );

      for (const item of response.items) {
        tracks.push(this.parseTrack(item.track));
      }

      if (!response.next) break;
      offset += limit;
    }

    return tracks;
  }

  async getFollowedArtists(credentials: SpotifyCredentials): Promise<SpotifyArtist[]> {
    const artists: SpotifyArtist[] = [];
    let after: string | null = null;
    const limit = 50;

    while (true) {
      const url = after
        ? `/me/following?type=artist&limit=${limit}&after=${after}`
        : `/me/following?type=artist&limit=${limit}`;

      const response = await this.spotifyRequest(credentials, url);

      for (const artist of response.artists.items) {
        artists.push({
          id: artist.id,
          name: artist.name,
          imageUrl: artist.images?.[0]?.url,
          genres: artist.genres,
          followers: artist.followers.total,
        });
      }

      if (!response.artists.next) break;
      after = response.artists.cursors?.after;
      if (!after) break;
    }

    return artists;
  }

  async getTopTracks(
    credentials: SpotifyCredentials,
    timeRange: 'short_term' | 'medium_term' | 'long_term' = 'medium_term'
  ): Promise<SpotifyTrack[]> {
    const response = await this.spotifyRequest(
      credentials,
      `/me/top/tracks?time_range=${timeRange}&limit=50`
    );

    return response.items.map((track: any) => this.parseTrack(track));
  }

  async getRecentlyPlayed(credentials: SpotifyCredentials): Promise<SpotifyTrack[]> {
    const response = await this.spotifyRequest(
      credentials,
      '/me/player/recently-played?limit=50'
    );

    return response.items.map((item: any) => this.parseTrack(item.track));
  }

  private parseTrack(track: any): SpotifyTrack {
    return {
      id: track.id,
      name: track.name,
      artists: track.artists.map((a: any) => ({ id: a.id, name: a.name })),
      album: {
        id: track.album.id,
        name: track.album.name,
        imageUrl: track.album.images?.[0]?.url,
        releaseDate: track.album.release_date,
      },
      durationMs: track.duration_ms,
      isrc: track.external_ids?.isrc,
      previewUrl: track.preview_url,
    };
  }

  private async spotifyRequest(
    credentials: SpotifyCredentials,
    endpoint: string
  ): Promise<any> {
    const response = await fetch(`${this.SPOTIFY_API}${endpoint}`, {
      headers: {
        Authorization: `Bearer ${credentials.accessToken}`,
      },
    });

    if (response.status === 429) {
      const retryAfter = parseInt(response.headers.get('Retry-After') || '1', 10);
      await new Promise(resolve => setTimeout(resolve, retryAfter * 1000));
      return this.spotifyRequest(credentials, endpoint);
    }

    if (!response.ok) {
      throw new Error(`Spotify API error: ${response.status}`);
    }

    return response.json();
  }

  // ============================================================================
  // Track Matching
  // ============================================================================

  async matchTrack(spotifyTrack: SpotifyTrack): Promise<TrackMatch> {
    // Try ISRC match first (most reliable)
    if (spotifyTrack.isrc) {
      const isrcMatch = await this.matchByIsrc(spotifyTrack.isrc);
      if (isrcMatch) {
        return {
          spotifyTrack,
          mycelixTrackId: isrcMatch,
          confidence: 1.0,
          matchMethod: 'isrc',
        };
      }
    }

    // Try metadata match
    const metadataMatch = await this.matchByMetadata(
      spotifyTrack.name,
      spotifyTrack.artists.map(a => a.name),
      spotifyTrack.album.name,
      spotifyTrack.durationMs
    );

    if (metadataMatch.trackId && metadataMatch.confidence >= 0.8) {
      return {
        spotifyTrack,
        mycelixTrackId: metadataMatch.trackId,
        confidence: metadataMatch.confidence,
        matchMethod: 'metadata',
      };
    }

    // No match found
    return {
      spotifyTrack,
      mycelixTrackId: null,
      confidence: 0,
      matchMethod: 'none',
    };
  }

  private async matchByIsrc(isrc: string): Promise<string | null> {
    // Would query database for ISRC match
    // For now, simulate
    try {
      const response = await fetch(`/api/tracks/by-isrc/${isrc}`);
      if (response.ok) {
        const data = await response.json();
        return data.trackId;
      }
    } catch {
      // No match
    }
    return null;
  }

  private async matchByMetadata(
    title: string,
    artists: string[],
    album: string,
    durationMs: number
  ): Promise<{ trackId: string | null; confidence: number }> {
    // Normalize strings for comparison
    const normalizedTitle = this.normalizeString(title);
    const normalizedArtists = artists.map(a => this.normalizeString(a));

    try {
      const response = await fetch('/api/tracks/search', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          title: normalizedTitle,
          artists: normalizedArtists,
          album,
          durationMs,
        }),
      });

      if (response.ok) {
        const matches = await response.json();
        if (matches.length > 0) {
          const bestMatch = matches[0];

          // Calculate confidence based on metadata similarity
          let confidence = 0;

          // Title similarity
          const titleSim = this.similarity(normalizedTitle, this.normalizeString(bestMatch.title));
          confidence += titleSim * 0.4;

          // Artist similarity
          const artistSim = this.maxSimilarity(
            normalizedArtists,
            bestMatch.artists.map((a: any) => this.normalizeString(a.name))
          );
          confidence += artistSim * 0.4;

          // Duration similarity (within 3 seconds = perfect)
          const durationDiff = Math.abs(bestMatch.durationMs - durationMs);
          const durationSim = durationDiff < 3000 ? 1 : Math.max(0, 1 - durationDiff / 10000);
          confidence += durationSim * 0.2;

          return { trackId: bestMatch.id, confidence };
        }
      }
    } catch {
      // Search failed
    }

    return { trackId: null, confidence: 0 };
  }

  private normalizeString(s: string): string {
    return s
      .toLowerCase()
      .replace(/[^\w\s]/g, '')
      .replace(/\s+/g, ' ')
      .trim();
  }

  private similarity(a: string, b: string): number {
    if (a === b) return 1;
    if (a.length === 0 || b.length === 0) return 0;

    // Levenshtein distance based similarity
    const matrix: number[][] = [];

    for (let i = 0; i <= a.length; i++) {
      matrix[i] = [i];
    }

    for (let j = 0; j <= b.length; j++) {
      matrix[0][j] = j;
    }

    for (let i = 1; i <= a.length; i++) {
      for (let j = 1; j <= b.length; j++) {
        const cost = a[i - 1] === b[j - 1] ? 0 : 1;
        matrix[i][j] = Math.min(
          matrix[i - 1][j] + 1,
          matrix[i][j - 1] + 1,
          matrix[i - 1][j - 1] + cost
        );
      }
    }

    const distance = matrix[a.length][b.length];
    return 1 - distance / Math.max(a.length, b.length);
  }

  private maxSimilarity(setA: string[], setB: string[]): number {
    let maxSim = 0;
    for (const a of setA) {
      for (const b of setB) {
        maxSim = Math.max(maxSim, this.similarity(a, b));
      }
    }
    return maxSim;
  }

  // ============================================================================
  // Full Import Process
  // ============================================================================

  async startImport(
    userId: string,
    credentials: SpotifyCredentials,
    options: {
      importPlaylists: boolean;
      importLikedSongs: boolean;
      importFollowedArtists: boolean;
      importHistory: boolean;
    },
    onProgress?: ProgressCallback
  ): Promise<string> {
    const jobId = `import_${userId}_${Date.now()}`;

    const progress: ImportProgress = {
      status: 'in_progress',
      phase: 'authenticating',
      totalItems: 0,
      processedItems: 0,
      matchedItems: 0,
      errors: [],
      startedAt: new Date(),
    };

    this.importJobs.set(jobId, progress);

    // Run import in background
    this.runImport(jobId, userId, credentials, options, onProgress).catch(error => {
      const progress = this.importJobs.get(jobId);
      if (progress) {
        progress.status = 'failed';
        progress.errors.push({
          type: 'fetch',
          message: error.message,
        });
      }
    });

    return jobId;
  }

  private async runImport(
    jobId: string,
    userId: string,
    credentials: SpotifyCredentials,
    options: {
      importPlaylists: boolean;
      importLikedSongs: boolean;
      importFollowedArtists: boolean;
      importHistory: boolean;
    },
    onProgress?: ProgressCallback
  ): Promise<void> {
    const progress = this.importJobs.get(jobId)!;
    const updateProgress = (phase: ImportPhase) => {
      progress.phase = phase;
      onProgress?.(progress);
    };

    const result: ImportResult = {
      playlistsImported: 0,
      tracksMatched: 0,
      tracksUnmatched: 0,
      artistsFollowed: 0,
      likedSongsImported: 0,
      historyImported: false,
    };

    try {
      // Fetch playlists
      if (options.importPlaylists) {
        updateProgress('fetching_playlists');

        const playlists = await this.getUserPlaylists(credentials);
        progress.totalItems += playlists.length;

        for (const playlist of playlists) {
          try {
            const tracks = await this.getPlaylistTracks(credentials, playlist.id);
            const matches = await Promise.all(tracks.map(t => this.matchTrack(t)));

            const matchedTracks = matches.filter(m => m.mycelixTrackId);

            // Create playlist in Mycelix
            await this.createMycelixPlaylist(
              userId,
              playlist.name,
              playlist.description,
              matchedTracks.map(m => m.mycelixTrackId!)
            );

            result.playlistsImported++;
            result.tracksMatched += matchedTracks.length;
            result.tracksUnmatched += matches.filter(m => !m.mycelixTrackId).length;

            progress.processedItems++;
            progress.matchedItems += matchedTracks.length;
            onProgress?.(progress);
          } catch (error: any) {
            progress.errors.push({
              type: 'create',
              itemId: playlist.id,
              itemName: playlist.name,
              message: error.message,
            });
          }
        }
      }

      // Import liked songs
      if (options.importLikedSongs) {
        updateProgress('fetching_library');

        const likedSongs = await this.getLikedSongs(credentials);
        progress.totalItems += likedSongs.length;

        const matches = await Promise.all(likedSongs.map(t => this.matchTrack(t)));
        const matchedTracks = matches.filter(m => m.mycelixTrackId);

        // Like tracks in Mycelix
        for (const match of matchedTracks) {
          await this.likeTrackInMycelix(userId, match.mycelixTrackId!);
          progress.processedItems++;
          progress.matchedItems++;
          onProgress?.(progress);
        }

        result.likedSongsImported = matchedTracks.length;
        result.tracksMatched += matchedTracks.length;
        result.tracksUnmatched += matches.filter(m => !m.mycelixTrackId).length;
      }

      // Import followed artists
      if (options.importFollowedArtists) {
        updateProgress('fetching_artists');

        const artists = await this.getFollowedArtists(credentials);
        progress.totalItems += artists.length;

        for (const artist of artists) {
          const mycelixArtistId = await this.findMycelixArtist(artist.name);
          if (mycelixArtistId) {
            await this.followArtistInMycelix(userId, mycelixArtistId);
            result.artistsFollowed++;
            progress.matchedItems++;
          }
          progress.processedItems++;
          onProgress?.(progress);
        }
      }

      // Import listening history
      if (options.importHistory) {
        updateProgress('fetching_history');

        const topTracks = await this.getTopTracks(credentials);
        const recentlyPlayed = await this.getRecentlyPlayed(credentials);

        // Store as listening preferences for recommendations
        await this.storeListeningPreferences(userId, topTracks, recentlyPlayed);
        result.historyImported = true;
      }

      progress.status = 'completed';
      progress.phase = 'complete';
      progress.completedAt = new Date();
      onProgress?.(progress);

    } catch (error: any) {
      progress.status = 'failed';
      progress.errors.push({
        type: 'fetch',
        message: error.message,
      });
      onProgress?.(progress);
    }
  }

  getImportProgress(jobId: string): ImportProgress | null {
    return this.importJobs.get(jobId) || null;
  }

  // ============================================================================
  // Mycelix Integration
  // ============================================================================

  private async createMycelixPlaylist(
    userId: string,
    name: string,
    description: string,
    trackIds: string[]
  ): Promise<string> {
    const response = await fetch('/api/playlists', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        userId,
        name: `${name} (from Spotify)`,
        description,
        trackIds,
        source: 'spotify_import',
      }),
    });

    const data = await response.json();
    return data.playlistId;
  }

  private async likeTrackInMycelix(userId: string, trackId: string): Promise<void> {
    await fetch(`/api/users/${userId}/library/tracks`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ trackId }),
    });
  }

  private async findMycelixArtist(name: string): Promise<string | null> {
    try {
      const response = await fetch(`/api/artists/search?q=${encodeURIComponent(name)}`);
      const data = await response.json();

      if (data.artists && data.artists.length > 0) {
        const normalizedSearch = this.normalizeString(name);
        const match = data.artists.find(
          (a: any) => this.similarity(this.normalizeString(a.name), normalizedSearch) >= 0.9
        );
        return match?.id || null;
      }
    } catch {
      // Search failed
    }
    return null;
  }

  private async followArtistInMycelix(userId: string, artistId: string): Promise<void> {
    await fetch(`/api/users/${userId}/following/artists`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ artistId }),
    });
  }

  private async storeListeningPreferences(
    userId: string,
    topTracks: SpotifyTrack[],
    recentlyPlayed: SpotifyTrack[]
  ): Promise<void> {
    // Extract genres and preferences for recommendation engine
    const artistIds = new Set<string>();
    const genres = new Set<string>();

    for (const track of [...topTracks, ...recentlyPlayed]) {
      for (const artist of track.artists) {
        artistIds.add(artist.id);
      }
    }

    // Store preferences for recommendation engine
    await fetch(`/api/users/${userId}/preferences/imported`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        source: 'spotify',
        topTrackCount: topTracks.length,
        recentPlayCount: recentlyPlayed.length,
        artistCount: artistIds.size,
        importedAt: new Date().toISOString(),
      }),
    });
  }
}

export const spotifyImport = new SpotifyImportService();
export default spotifyImport;
