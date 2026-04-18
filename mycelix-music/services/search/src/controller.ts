// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Search Controller
 *
 * Handles search queries with full-text search, autocomplete, and fuzzy matching.
 */

import { Request, Response } from 'express';
import { Client } from '@elastic/elasticsearch';
import { z } from 'zod';
import { config } from './config';
import { createLogger } from './logger';
import { SongDocument, ArtistDocument, PlaylistDocument } from './indexer';

const logger = createLogger('search-controller');

const searchQuerySchema = z.object({
  q: z.string().min(1).max(200),
  type: z.enum(['all', 'songs', 'artists', 'playlists']).optional().default('all'),
  genre: z.string().optional(),
  limit: z.coerce.number().min(1).max(100).optional().default(20),
  offset: z.coerce.number().min(0).optional().default(0),
});

const autocompleteSchema = z.object({
  q: z.string().min(1).max(100),
  limit: z.coerce.number().min(1).max(10).optional().default(5),
});

export class SearchController {
  constructor(private client: Client) {}

  async search(req: Request, res: Response) {
    try {
      const query = searchQuerySchema.parse(req.query);
      const { q, type, genre, limit, offset } = query;

      const results: {
        songs: any[];
        artists: any[];
        playlists: any[];
        total: { songs: number; artists: number; playlists: number };
      } = {
        songs: [],
        artists: [],
        playlists: [],
        total: { songs: 0, artists: 0, playlists: 0 },
      };

      // Build multi-match query
      const baseQuery = {
        bool: {
          should: [
            {
              multi_match: {
                query: q,
                fields: ['title^3', 'artist^2', 'name^3', 'album', 'lyrics', 'description', 'bio'],
                type: 'best_fields',
                fuzziness: config.search.fuzziness,
              },
            },
            {
              multi_match: {
                query: q,
                fields: ['title.autocomplete', 'name.autocomplete'],
                type: 'phrase_prefix',
              },
            },
          ],
          minimum_should_match: 1,
          ...(genre && {
            filter: [
              { term: { genre: genre.toLowerCase() } },
            ],
          }),
        },
      };

      // Search songs
      if (type === 'all' || type === 'songs') {
        const songResults = await this.client.search({
          index: config.indices.songs,
          body: {
            query: baseQuery,
            highlight: {
              fields: {
                title: {},
                lyrics: { fragment_size: config.search.highlightFragmentSize },
              },
            },
            sort: [
              { _score: 'desc' },
              { playCount: 'desc' },
            ],
            from: offset,
            size: limit,
          },
        });

        results.songs = songResults.hits.hits.map((hit: any) => ({
          ...hit._source,
          _score: hit._score,
          _highlight: hit.highlight,
        }));
        results.total.songs = typeof songResults.hits.total === 'number'
          ? songResults.hits.total
          : songResults.hits.total?.value || 0;
      }

      // Search artists
      if (type === 'all' || type === 'artists') {
        const artistResults = await this.client.search({
          index: config.indices.artists,
          body: {
            query: {
              bool: {
                should: [
                  {
                    match: {
                      name: {
                        query: q,
                        fuzziness: config.search.fuzziness,
                        boost: 3,
                      },
                    },
                  },
                  {
                    match: {
                      'name.autocomplete': {
                        query: q,
                        boost: 2,
                      },
                    },
                  },
                  {
                    match: {
                      bio: q,
                    },
                  },
                ],
                minimum_should_match: 1,
              },
            },
            sort: [
              { _score: 'desc' },
              { followerCount: 'desc' },
            ],
            from: offset,
            size: type === 'artists' ? limit : Math.min(limit, 10),
          },
        });

        results.artists = artistResults.hits.hits.map((hit: any) => ({
          ...hit._source,
          _score: hit._score,
        }));
        results.total.artists = typeof artistResults.hits.total === 'number'
          ? artistResults.hits.total
          : artistResults.hits.total?.value || 0;
      }

      // Search playlists
      if (type === 'all' || type === 'playlists') {
        const playlistResults = await this.client.search({
          index: config.indices.playlists,
          body: {
            query: {
              bool: {
                must: [
                  { term: { isPublic: true } },
                ],
                should: [
                  {
                    match: {
                      name: {
                        query: q,
                        fuzziness: config.search.fuzziness,
                        boost: 3,
                      },
                    },
                  },
                  {
                    match: {
                      description: q,
                    },
                  },
                ],
                minimum_should_match: 1,
              },
            },
            sort: [
              { _score: 'desc' },
              { followerCount: 'desc' },
            ],
            from: offset,
            size: type === 'playlists' ? limit : Math.min(limit, 10),
          },
        });

        results.playlists = playlistResults.hits.hits.map((hit: any) => ({
          ...hit._source,
          _score: hit._score,
        }));
        results.total.playlists = typeof playlistResults.hits.total === 'number'
          ? playlistResults.hits.total
          : playlistResults.hits.total?.value || 0;
      }

      res.json({
        success: true,
        data: results,
        meta: {
          query: q,
          type,
          limit,
          offset,
        },
      });
    } catch (error) {
      if (error instanceof z.ZodError) {
        res.status(400).json({
          success: false,
          error: { code: 'VALIDATION_ERROR', details: error.errors },
        });
        return;
      }

      logger.error({ error }, 'Search failed');
      res.status(500).json({
        success: false,
        error: { code: 'SEARCH_ERROR', message: 'Search failed' },
      });
    }
  }

  async autocomplete(req: Request, res: Response) {
    try {
      const query = autocompleteSchema.parse(req.query);
      const { q, limit } = query;

      // Multi-index search for autocomplete
      const results = await this.client.search({
        index: [config.indices.songs, config.indices.artists, config.indices.playlists],
        body: {
          query: {
            bool: {
              should: [
                {
                  match: {
                    'title.autocomplete': {
                      query: q,
                      boost: 3,
                    },
                  },
                },
                {
                  match: {
                    'name.autocomplete': {
                      query: q,
                      boost: 3,
                    },
                  },
                },
                {
                  prefix: {
                    'title.keyword': {
                      value: q,
                      boost: 2,
                    },
                  },
                },
                {
                  prefix: {
                    'name.keyword': {
                      value: q,
                      boost: 2,
                    },
                  },
                },
              ],
            },
          },
          _source: ['id', 'address', 'title', 'name', 'artist', 'coverArt', 'avatar'],
          size: limit * 3, // Get more to filter by type
        },
      });

      // Group by type
      const suggestions = results.hits.hits.map((hit: any) => {
        const source = hit._source;
        const index = hit._index;

        if (index.includes('songs')) {
          return {
            type: 'song',
            id: source.id,
            title: source.title,
            subtitle: source.artist,
            image: source.coverArt,
          };
        } else if (index.includes('artists')) {
          return {
            type: 'artist',
            id: source.address,
            title: source.name,
            subtitle: 'Artist',
            image: source.avatar,
          };
        } else {
          return {
            type: 'playlist',
            id: source.id,
            title: source.name,
            subtitle: 'Playlist',
            image: source.coverImage,
          };
        }
      }).slice(0, limit);

      res.json({
        success: true,
        data: suggestions,
      });
    } catch (error) {
      if (error instanceof z.ZodError) {
        res.status(400).json({
          success: false,
          error: { code: 'VALIDATION_ERROR', details: error.errors },
        });
        return;
      }

      logger.error({ error }, 'Autocomplete failed');
      res.status(500).json({
        success: false,
        error: { code: 'AUTOCOMPLETE_ERROR', message: 'Autocomplete failed' },
      });
    }
  }

  async findSimilar(req: Request, res: Response) {
    try {
      const { songId } = req.params;
      const limit = Math.min(parseInt(req.query.limit as string) || 10, 50);

      // Get the source song
      const song = await this.client.get({
        index: config.indices.songs,
        id: songId,
      }).catch(() => null);

      if (!song) {
        res.status(404).json({
          success: false,
          error: { code: 'NOT_FOUND', message: 'Song not found' },
        });
        return;
      }

      const source = song._source as SongDocument;

      // Use "more like this" query combined with genre/feature matching
      const results = await this.client.search({
        index: config.indices.songs,
        body: {
          query: {
            bool: {
              must_not: [
                { term: { id: songId } },
              ],
              should: [
                {
                  more_like_this: {
                    fields: ['title', 'artist', 'tags', 'genre'],
                    like: [{ _index: config.indices.songs, _id: songId }],
                    min_term_freq: 1,
                    min_doc_freq: 1,
                    boost: 2,
                  },
                },
                ...(source.genre ? [{ term: { genre: { value: source.genre, boost: 3 } } }] : []),
                ...(source.audioFeatures ? [
                  {
                    script_score: {
                      query: { match_all: {} },
                      script: {
                        source: `
                          double score = 0;
                          if (doc['audioFeatures.energy'].size() > 0) {
                            score += 1 - Math.abs(doc['audioFeatures.energy'].value - params.energy);
                          }
                          if (doc['audioFeatures.danceability'].size() > 0) {
                            score += 1 - Math.abs(doc['audioFeatures.danceability'].value - params.danceability);
                          }
                          return score;
                        `,
                        params: {
                          energy: source.audioFeatures.energy || 0.5,
                          danceability: source.audioFeatures.danceability || 0.5,
                        },
                      },
                    },
                  },
                ] : []),
              ],
              minimum_should_match: 1,
            },
          },
          size: limit,
        },
      });

      const similar = results.hits.hits.map((hit: any) => ({
        ...hit._source,
        _score: hit._score,
      }));

      res.json({
        success: true,
        data: similar,
      });
    } catch (error) {
      logger.error({ error }, 'Find similar failed');
      res.status(500).json({
        success: false,
        error: { code: 'SEARCH_ERROR', message: 'Find similar failed' },
      });
    }
  }

  // Index management endpoints
  async indexSong(req: Request, res: Response) {
    try {
      const song = req.body as SongDocument;
      await this.client.index({
        index: config.indices.songs,
        id: song.id,
        body: song,
        refresh: true,
      });
      res.json({ success: true });
    } catch (error) {
      logger.error({ error }, 'Index song failed');
      res.status(500).json({ success: false });
    }
  }

  async indexArtist(req: Request, res: Response) {
    try {
      const artist = req.body as ArtistDocument;
      await this.client.index({
        index: config.indices.artists,
        id: artist.address,
        body: artist,
        refresh: true,
      });
      res.json({ success: true });
    } catch (error) {
      logger.error({ error }, 'Index artist failed');
      res.status(500).json({ success: false });
    }
  }

  async indexPlaylist(req: Request, res: Response) {
    try {
      const playlist = req.body as PlaylistDocument;
      await this.client.index({
        index: config.indices.playlists,
        id: playlist.id,
        body: playlist,
        refresh: true,
      });
      res.json({ success: true });
    } catch (error) {
      logger.error({ error }, 'Index playlist failed');
      res.status(500).json({ success: false });
    }
  }

  async removeFromIndex(req: Request, res: Response) {
    try {
      const { type, id } = req.params;
      const index = type === 'song' ? config.indices.songs
        : type === 'artist' ? config.indices.artists
        : config.indices.playlists;

      await this.client.delete({
        index,
        id,
        refresh: true,
      });
      res.json({ success: true });
    } catch (error) {
      res.json({ success: true }); // Ignore not found
    }
  }
}
