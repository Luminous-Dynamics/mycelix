// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Index Manager
 *
 * Manages Elasticsearch indices for songs, artists, and playlists.
 */

import { Client } from '@elastic/elasticsearch';
import { config } from './config';
import { createLogger } from './logger';

const logger = createLogger('indexer');

// Index mappings
const SONG_MAPPING = {
  properties: {
    id: { type: 'keyword' },
    title: {
      type: 'text',
      analyzer: 'standard',
      fields: {
        keyword: { type: 'keyword' },
        autocomplete: {
          type: 'text',
          analyzer: 'autocomplete',
          search_analyzer: 'standard',
        },
      },
    },
    artist: {
      type: 'text',
      analyzer: 'standard',
      fields: {
        keyword: { type: 'keyword' },
      },
    },
    artistAddress: { type: 'keyword' },
    album: { type: 'text' },
    genre: { type: 'keyword' },
    tags: { type: 'keyword' },
    lyrics: { type: 'text', analyzer: 'standard' },
    duration: { type: 'integer' },
    playCount: { type: 'long' },
    likeCount: { type: 'long' },
    releaseDate: { type: 'date' },
    createdAt: { type: 'date' },
    coverArt: { type: 'keyword', index: false },
    audioUrl: { type: 'keyword', index: false },
    isExplicit: { type: 'boolean' },
    bpm: { type: 'float' },
    key: { type: 'keyword' },
    // For "more like this" queries
    audioFeatures: {
      properties: {
        energy: { type: 'float' },
        danceability: { type: 'float' },
        valence: { type: 'float' },
        acousticness: { type: 'float' },
        instrumentalness: { type: 'float' },
      },
    },
  },
};

const ARTIST_MAPPING = {
  properties: {
    address: { type: 'keyword' },
    name: {
      type: 'text',
      analyzer: 'standard',
      fields: {
        keyword: { type: 'keyword' },
        autocomplete: {
          type: 'text',
          analyzer: 'autocomplete',
          search_analyzer: 'standard',
        },
      },
    },
    bio: { type: 'text' },
    genres: { type: 'keyword' },
    followerCount: { type: 'long' },
    songCount: { type: 'integer' },
    totalStreams: { type: 'long' },
    isVerified: { type: 'boolean' },
    avatar: { type: 'keyword', index: false },
    createdAt: { type: 'date' },
  },
};

const PLAYLIST_MAPPING = {
  properties: {
    id: { type: 'keyword' },
    name: {
      type: 'text',
      analyzer: 'standard',
      fields: {
        keyword: { type: 'keyword' },
        autocomplete: {
          type: 'text',
          analyzer: 'autocomplete',
          search_analyzer: 'standard',
        },
      },
    },
    description: { type: 'text' },
    ownerAddress: { type: 'keyword' },
    ownerName: { type: 'text' },
    songCount: { type: 'integer' },
    followerCount: { type: 'long' },
    isPublic: { type: 'boolean' },
    genres: { type: 'keyword' },
    coverImage: { type: 'keyword', index: false },
    createdAt: { type: 'date' },
  },
};

const INDEX_SETTINGS = {
  number_of_shards: 1,
  number_of_replicas: 1,
  analysis: {
    analyzer: {
      autocomplete: {
        type: 'custom',
        tokenizer: 'standard',
        filter: ['lowercase', 'autocomplete_filter'],
      },
    },
    filter: {
      autocomplete_filter: {
        type: 'edge_ngram',
        min_gram: 1,
        max_gram: 20,
      },
    },
  },
};

export interface SongDocument {
  id: string;
  title: string;
  artist: string;
  artistAddress: string;
  album?: string;
  genre?: string;
  tags?: string[];
  lyrics?: string;
  duration: number;
  playCount: number;
  likeCount: number;
  releaseDate?: string;
  createdAt: string;
  coverArt?: string;
  audioUrl?: string;
  isExplicit?: boolean;
  bpm?: number;
  key?: string;
  audioFeatures?: {
    energy?: number;
    danceability?: number;
    valence?: number;
    acousticness?: number;
    instrumentalness?: number;
  };
}

export interface ArtistDocument {
  address: string;
  name: string;
  bio?: string;
  genres?: string[];
  followerCount: number;
  songCount: number;
  totalStreams: number;
  isVerified?: boolean;
  avatar?: string;
  createdAt: string;
}

export interface PlaylistDocument {
  id: string;
  name: string;
  description?: string;
  ownerAddress: string;
  ownerName: string;
  songCount: number;
  followerCount: number;
  isPublic: boolean;
  genres?: string[];
  coverImage?: string;
  createdAt: string;
}

export class IndexManager {
  constructor(private client: Client) {}

  async ensureIndices(): Promise<void> {
    const indices = [
      { name: config.indices.songs, mapping: SONG_MAPPING },
      { name: config.indices.artists, mapping: ARTIST_MAPPING },
      { name: config.indices.playlists, mapping: PLAYLIST_MAPPING },
    ];

    for (const { name, mapping } of indices) {
      const exists = await this.client.indices.exists({ index: name });

      if (!exists) {
        logger.info({ index: name }, 'Creating index');
        await this.client.indices.create({
          index: name,
          body: {
            settings: INDEX_SETTINGS,
            mappings: mapping,
          },
        });
      }
    }
  }

  async indexSong(song: SongDocument): Promise<void> {
    await this.client.index({
      index: config.indices.songs,
      id: song.id,
      body: song,
      refresh: true,
    });
    logger.debug({ songId: song.id }, 'Indexed song');
  }

  async indexArtist(artist: ArtistDocument): Promise<void> {
    await this.client.index({
      index: config.indices.artists,
      id: artist.address,
      body: artist,
      refresh: true,
    });
    logger.debug({ artistAddress: artist.address }, 'Indexed artist');
  }

  async indexPlaylist(playlist: PlaylistDocument): Promise<void> {
    await this.client.index({
      index: config.indices.playlists,
      id: playlist.id,
      body: playlist,
      refresh: true,
    });
    logger.debug({ playlistId: playlist.id }, 'Indexed playlist');
  }

  async removeSong(id: string): Promise<void> {
    await this.client.delete({
      index: config.indices.songs,
      id,
      refresh: true,
    }).catch(() => {});
  }

  async removeArtist(address: string): Promise<void> {
    await this.client.delete({
      index: config.indices.artists,
      id: address,
      refresh: true,
    }).catch(() => {});
  }

  async removePlaylist(id: string): Promise<void> {
    await this.client.delete({
      index: config.indices.playlists,
      id,
      refresh: true,
    }).catch(() => {});
  }

  async getStats(): Promise<{
    songs: number;
    artists: number;
    playlists: number;
  }> {
    const [songs, artists, playlists] = await Promise.all([
      this.client.count({ index: config.indices.songs }),
      this.client.count({ index: config.indices.artists }),
      this.client.count({ index: config.indices.playlists }),
    ]);

    return {
      songs: songs.count,
      artists: artists.count,
      playlists: playlists.count,
    };
  }

  async rebuildIndex(type: 'songs' | 'artists' | 'playlists'): Promise<void> {
    const indexName = config.indices[type];
    const tempIndex = `${indexName}_temp`;

    // Create temp index with same mapping
    const mapping = type === 'songs' ? SONG_MAPPING
      : type === 'artists' ? ARTIST_MAPPING
      : PLAYLIST_MAPPING;

    await this.client.indices.create({
      index: tempIndex,
      body: {
        settings: INDEX_SETTINGS,
        mappings: mapping,
      },
    });

    // Reindex data
    await this.client.reindex({
      body: {
        source: { index: indexName },
        dest: { index: tempIndex },
      },
    });

    // Delete old index
    await this.client.indices.delete({ index: indexName });

    // Rename temp to original
    await this.client.indices.putAlias({
      index: tempIndex,
      name: indexName,
    });

    logger.info({ index: indexName }, 'Index rebuilt');
  }
}
