// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Discord Bot Integration
 *
 * Discord bot for Mycelix featuring now playing status, listening parties,
 * artist announcements, and community features.
 */

import {
  Client,
  GatewayIntentBits,
  SlashCommandBuilder,
  EmbedBuilder,
  ActionRowBuilder,
  ButtonBuilder,
  ButtonStyle,
  CommandInteraction,
  ButtonInteraction,
  GuildMember,
  VoiceState,
  TextChannel,
} from 'discord.js';

// ============================================================================
// Types
// ============================================================================

export interface DiscordGuildConfig {
  guildId: string;
  nowPlayingChannelId?: string;
  announcementsChannelId?: string;
  listeningPartyChannelId?: string;
  linkedArtistIds: string[];
  enableNowPlaying: boolean;
  enableRichPresence: boolean;
}

export interface ListeningParty {
  id: string;
  guildId: string;
  voiceChannelId: string;
  textChannelId: string;
  hostUserId: string;
  hostDiscordId: string;
  currentTrackId: string | null;
  participants: string[];
  isActive: boolean;
  startedAt: Date;
  trackQueue: string[];
}

export interface UserLink {
  discordId: string;
  mycelixUserId: string;
  linkedAt: Date;
  showNowPlaying: boolean;
}

export interface NowPlayingUpdate {
  userId: string;
  trackId: string;
  title: string;
  artist: string;
  album: string;
  artworkUrl: string;
  duration: number;
  position: number;
  isPlaying: boolean;
}

// ============================================================================
// Discord Bot Service
// ============================================================================

class DiscordBotService {
  private client: Client;
  private guildConfigs: Map<string, DiscordGuildConfig> = new Map();
  private userLinks: Map<string, UserLink> = new Map();
  private listeningParties: Map<string, ListeningParty> = new Map();
  private isReady = false;

  constructor() {
    this.client = new Client({
      intents: [
        GatewayIntentBits.Guilds,
        GatewayIntentBits.GuildMessages,
        GatewayIntentBits.GuildVoiceStates,
        GatewayIntentBits.GuildMembers,
      ],
    });

    this.setupEventHandlers();
  }

  // ============================================================================
  // Initialization
  // ============================================================================

  async start(token: string): Promise<void> {
    await this.client.login(token);
    await this.registerCommands();
  }

  private setupEventHandlers(): void {
    this.client.once('ready', () => {
      console.log(`Discord bot logged in as ${this.client.user?.tag}`);
      this.isReady = true;
    });

    this.client.on('interactionCreate', async (interaction) => {
      if (interaction.isCommand()) {
        await this.handleCommand(interaction as CommandInteraction);
      } else if (interaction.isButton()) {
        await this.handleButton(interaction as ButtonInteraction);
      }
    });

    this.client.on('voiceStateUpdate', async (oldState, newState) => {
      await this.handleVoiceStateUpdate(oldState, newState);
    });
  }

  private async registerCommands(): Promise<void> {
    const commands = [
      new SlashCommandBuilder()
        .setName('link')
        .setDescription('Link your Discord account to Mycelix')
        .addStringOption(option =>
          option.setName('code')
            .setDescription('Your Mycelix linking code')
            .setRequired(true)
        ),

      new SlashCommandBuilder()
        .setName('unlink')
        .setDescription('Unlink your Discord account from Mycelix'),

      new SlashCommandBuilder()
        .setName('nowplaying')
        .setDescription('Show what you\'re currently listening to'),

      new SlashCommandBuilder()
        .setName('np')
        .setDescription('Show what you\'re currently listening to (shortcut)'),

      new SlashCommandBuilder()
        .setName('party')
        .setDescription('Start or join a listening party')
        .addSubcommand(subcommand =>
          subcommand.setName('start')
            .setDescription('Start a new listening party')
        )
        .addSubcommand(subcommand =>
          subcommand.setName('join')
            .setDescription('Join an existing listening party')
            .addStringOption(option =>
              option.setName('host')
                .setDescription('Username of the party host')
                .setRequired(true)
            )
        )
        .addSubcommand(subcommand =>
          subcommand.setName('leave')
            .setDescription('Leave the current listening party')
        )
        .addSubcommand(subcommand =>
          subcommand.setName('end')
            .setDescription('End your listening party (host only)')
        ),

      new SlashCommandBuilder()
        .setName('search')
        .setDescription('Search for tracks on Mycelix')
        .addStringOption(option =>
          option.setName('query')
            .setDescription('Search query')
            .setRequired(true)
        ),

      new SlashCommandBuilder()
        .setName('artist')
        .setDescription('Get information about an artist')
        .addStringOption(option =>
          option.setName('name')
            .setDescription('Artist name')
            .setRequired(true)
        ),

      new SlashCommandBuilder()
        .setName('top')
        .setDescription('Show your top tracks or artists')
        .addStringOption(option =>
          option.setName('type')
            .setDescription('What to show')
            .setRequired(true)
            .addChoices(
              { name: 'Tracks', value: 'tracks' },
              { name: 'Artists', value: 'artists' }
            )
        )
        .addStringOption(option =>
          option.setName('period')
            .setDescription('Time period')
            .addChoices(
              { name: 'This Week', value: 'week' },
              { name: 'This Month', value: 'month' },
              { name: 'All Time', value: 'all' }
            )
        ),

      new SlashCommandBuilder()
        .setName('setup')
        .setDescription('Configure Mycelix bot for this server (admin only)')
        .addChannelOption(option =>
          option.setName('announcements')
            .setDescription('Channel for artist announcements')
        )
        .addChannelOption(option =>
          option.setName('nowplaying')
            .setDescription('Channel for now playing updates')
        ),
    ];

    try {
      await this.client.application?.commands.set(
        commands.map(c => c.toJSON())
      );
      console.log('Discord commands registered');
    } catch (error) {
      console.error('Failed to register commands:', error);
    }
  }

  // ============================================================================
  // Command Handlers
  // ============================================================================

  private async handleCommand(interaction: CommandInteraction): Promise<void> {
    const { commandName } = interaction;

    try {
      switch (commandName) {
        case 'link':
          await this.handleLinkCommand(interaction);
          break;
        case 'unlink':
          await this.handleUnlinkCommand(interaction);
          break;
        case 'nowplaying':
        case 'np':
          await this.handleNowPlayingCommand(interaction);
          break;
        case 'party':
          await this.handlePartyCommand(interaction);
          break;
        case 'search':
          await this.handleSearchCommand(interaction);
          break;
        case 'artist':
          await this.handleArtistCommand(interaction);
          break;
        case 'top':
          await this.handleTopCommand(interaction);
          break;
        case 'setup':
          await this.handleSetupCommand(interaction);
          break;
        default:
          await interaction.reply({ content: 'Unknown command', ephemeral: true });
      }
    } catch (error) {
      console.error(`Command error (${commandName}):`, error);
      await interaction.reply({
        content: 'An error occurred while processing your command.',
        ephemeral: true,
      });
    }
  }

  private async handleLinkCommand(interaction: CommandInteraction): Promise<void> {
    const code = interaction.options.get('code')?.value as string;

    // Verify linking code with Mycelix API
    const response = await fetch('/api/integrations/discord/verify-link', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ code, discordId: interaction.user.id }),
    });

    if (!response.ok) {
      await interaction.reply({
        content: 'Invalid or expired linking code. Get a new code from your Mycelix settings.',
        ephemeral: true,
      });
      return;
    }

    const data = await response.json();

    this.userLinks.set(interaction.user.id, {
      discordId: interaction.user.id,
      mycelixUserId: data.userId,
      linkedAt: new Date(),
      showNowPlaying: true,
    });

    const embed = new EmbedBuilder()
      .setColor('#7C3AED')
      .setTitle('Account Linked!')
      .setDescription(`Your Discord account is now linked to Mycelix as **${data.username}**`)
      .addFields(
        { name: 'Now Playing', value: 'Use `/nowplaying` to share what you\'re listening to', inline: true },
        { name: 'Listening Parties', value: 'Use `/party start` to host a listening party', inline: true }
      )
      .setFooter({ text: 'Mycelix' });

    await interaction.reply({ embeds: [embed], ephemeral: true });
  }

  private async handleUnlinkCommand(interaction: CommandInteraction): Promise<void> {
    if (!this.userLinks.has(interaction.user.id)) {
      await interaction.reply({
        content: 'Your account is not linked to Mycelix.',
        ephemeral: true,
      });
      return;
    }

    this.userLinks.delete(interaction.user.id);

    await interaction.reply({
      content: 'Your Discord account has been unlinked from Mycelix.',
      ephemeral: true,
    });
  }

  private async handleNowPlayingCommand(interaction: CommandInteraction): Promise<void> {
    const userLink = this.userLinks.get(interaction.user.id);

    if (!userLink) {
      await interaction.reply({
        content: 'Link your account first with `/link`',
        ephemeral: true,
      });
      return;
    }

    // Fetch current playback from Mycelix
    const response = await fetch(`/api/users/${userLink.mycelixUserId}/now-playing`);

    if (!response.ok) {
      await interaction.reply({
        content: 'Nothing playing right now.',
        ephemeral: true,
      });
      return;
    }

    const nowPlaying = await response.json();

    const embed = this.createNowPlayingEmbed(nowPlaying, interaction.user.username);
    const row = this.createNowPlayingButtons(nowPlaying.trackId);

    await interaction.reply({ embeds: [embed], components: [row] });
  }

  private async handlePartyCommand(interaction: CommandInteraction): Promise<void> {
    const subcommand = interaction.options.data[0]?.name;
    const userLink = this.userLinks.get(interaction.user.id);

    if (!userLink) {
      await interaction.reply({
        content: 'Link your account first with `/link`',
        ephemeral: true,
      });
      return;
    }

    const member = interaction.member as GuildMember;
    const voiceChannel = member.voice.channel;

    switch (subcommand) {
      case 'start':
        if (!voiceChannel) {
          await interaction.reply({
            content: 'Join a voice channel first to start a listening party!',
            ephemeral: true,
          });
          return;
        }

        const partyId = `party_${interaction.guildId}_${Date.now()}`;
        const party: ListeningParty = {
          id: partyId,
          guildId: interaction.guildId!,
          voiceChannelId: voiceChannel.id,
          textChannelId: interaction.channelId,
          hostUserId: userLink.mycelixUserId,
          hostDiscordId: interaction.user.id,
          currentTrackId: null,
          participants: [userLink.mycelixUserId],
          isActive: true,
          startedAt: new Date(),
          trackQueue: [],
        };

        this.listeningParties.set(partyId, party);

        const startEmbed = new EmbedBuilder()
          .setColor('#7C3AED')
          .setTitle('Listening Party Started!')
          .setDescription(`${interaction.user.username} started a listening party in ${voiceChannel.name}`)
          .addFields(
            { name: 'Join', value: `Join the voice channel and use \`/party join ${interaction.user.username}\``, inline: false }
          )
          .setFooter({ text: 'Mycelix Listening Party' });

        await interaction.reply({ embeds: [startEmbed] });
        break;

      case 'join':
        const hostName = interaction.options.get('host')?.value as string;
        // Find party by host
        const existingParty = Array.from(this.listeningParties.values()).find(
          p => p.isActive && p.guildId === interaction.guildId
        );

        if (!existingParty) {
          await interaction.reply({
            content: 'No active listening party found.',
            ephemeral: true,
          });
          return;
        }

        existingParty.participants.push(userLink.mycelixUserId);

        // Sync playback with Mycelix
        await fetch('/api/circles/sync-playback', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            userId: userLink.mycelixUserId,
            hostUserId: existingParty.hostUserId,
          }),
        });

        await interaction.reply({
          content: `You joined the listening party! Your playback is now synced.`,
          ephemeral: true,
        });
        break;

      case 'leave':
        const currentParty = Array.from(this.listeningParties.values()).find(
          p => p.participants.includes(userLink.mycelixUserId) && p.isActive
        );

        if (!currentParty) {
          await interaction.reply({
            content: 'You\'re not in a listening party.',
            ephemeral: true,
          });
          return;
        }

        currentParty.participants = currentParty.participants.filter(
          p => p !== userLink.mycelixUserId
        );

        await interaction.reply({
          content: 'You left the listening party.',
          ephemeral: true,
        });
        break;

      case 'end':
        const hostedParty = Array.from(this.listeningParties.values()).find(
          p => p.hostDiscordId === interaction.user.id && p.isActive
        );

        if (!hostedParty) {
          await interaction.reply({
            content: 'You\'re not hosting a listening party.',
            ephemeral: true,
          });
          return;
        }

        hostedParty.isActive = false;

        const endEmbed = new EmbedBuilder()
          .setColor('#EF4444')
          .setTitle('Listening Party Ended')
          .setDescription(`${interaction.user.username}'s listening party has ended.`)
          .addFields(
            { name: 'Duration', value: this.formatDuration(Date.now() - hostedParty.startedAt.getTime()), inline: true },
            { name: 'Participants', value: hostedParty.participants.length.toString(), inline: true }
          );

        await interaction.reply({ embeds: [endEmbed] });
        break;
    }
  }

  private async handleSearchCommand(interaction: CommandInteraction): Promise<void> {
    const query = interaction.options.get('query')?.value as string;

    await interaction.deferReply();

    const response = await fetch(`/api/search?q=${encodeURIComponent(query)}&limit=5`);
    const results = await response.json();

    if (!results.tracks || results.tracks.length === 0) {
      await interaction.editReply('No results found.');
      return;
    }

    const embed = new EmbedBuilder()
      .setColor('#7C3AED')
      .setTitle(`Search Results for "${query}"`)
      .setDescription(
        results.tracks.map((track: any, i: number) =>
          `**${i + 1}.** ${track.title} - ${track.artist}\n`
        ).join('')
      )
      .setFooter({ text: 'Mycelix' });

    const rows: ActionRowBuilder<ButtonBuilder>[] = [];
    const buttons = results.tracks.slice(0, 5).map((track: any, i: number) =>
      new ButtonBuilder()
        .setCustomId(`play_${track.id}`)
        .setLabel(`Play #${i + 1}`)
        .setStyle(ButtonStyle.Primary)
    );

    rows.push(new ActionRowBuilder<ButtonBuilder>().addComponents(buttons));

    await interaction.editReply({ embeds: [embed], components: rows });
  }

  private async handleArtistCommand(interaction: CommandInteraction): Promise<void> {
    const name = interaction.options.get('name')?.value as string;

    await interaction.deferReply();

    const response = await fetch(`/api/artists/search?q=${encodeURIComponent(name)}`);
    const results = await response.json();

    if (!results.artists || results.artists.length === 0) {
      await interaction.editReply('Artist not found.');
      return;
    }

    const artist = results.artists[0];

    const embed = new EmbedBuilder()
      .setColor('#7C3AED')
      .setTitle(artist.name)
      .setThumbnail(artist.imageUrl)
      .addFields(
        { name: 'Followers', value: this.formatNumber(artist.followers), inline: true },
        { name: 'Tracks', value: artist.trackCount.toString(), inline: true },
        { name: 'Genres', value: artist.genres.slice(0, 3).join(', ') || 'N/A', inline: true }
      )
      .setDescription(artist.bio?.substring(0, 200) || 'No bio available.')
      .setFooter({ text: 'Mycelix' });

    const row = new ActionRowBuilder<ButtonBuilder>().addComponents(
      new ButtonBuilder()
        .setLabel('View on Mycelix')
        .setStyle(ButtonStyle.Link)
        .setURL(`https://mycelix.io/artist/${artist.id}`),
      new ButtonBuilder()
        .setCustomId(`follow_${artist.id}`)
        .setLabel('Follow')
        .setStyle(ButtonStyle.Secondary)
    );

    await interaction.editReply({ embeds: [embed], components: [row] });
  }

  private async handleTopCommand(interaction: CommandInteraction): Promise<void> {
    const type = interaction.options.get('type')?.value as string;
    const period = (interaction.options.get('period')?.value as string) || 'month';

    const userLink = this.userLinks.get(interaction.user.id);

    if (!userLink) {
      await interaction.reply({
        content: 'Link your account first with `/link`',
        ephemeral: true,
      });
      return;
    }

    await interaction.deferReply();

    const response = await fetch(
      `/api/users/${userLink.mycelixUserId}/top/${type}?period=${period}`
    );
    const data = await response.json();

    const items = type === 'tracks' ? data.tracks : data.artists;

    if (!items || items.length === 0) {
      await interaction.editReply('No listening data yet.');
      return;
    }

    const periodLabels: Record<string, string> = {
      week: 'This Week',
      month: 'This Month',
      all: 'All Time',
    };

    const embed = new EmbedBuilder()
      .setColor('#7C3AED')
      .setTitle(`Your Top ${type === 'tracks' ? 'Tracks' : 'Artists'} - ${periodLabels[period]}`)
      .setDescription(
        items.slice(0, 10).map((item: any, i: number) =>
          type === 'tracks'
            ? `**${i + 1}.** ${item.title} - ${item.artist} (${item.playCount} plays)`
            : `**${i + 1}.** ${item.name} (${item.playCount} plays)`
        ).join('\n')
      )
      .setFooter({ text: 'Mycelix' });

    await interaction.editReply({ embeds: [embed] });
  }

  private async handleSetupCommand(interaction: CommandInteraction): Promise<void> {
    const member = interaction.member as GuildMember;

    if (!member.permissions.has('ManageGuild')) {
      await interaction.reply({
        content: 'You need Manage Server permission to configure the bot.',
        ephemeral: true,
      });
      return;
    }

    const announcementsChannel = interaction.options.get('announcements')?.channel;
    const nowPlayingChannel = interaction.options.get('nowplaying')?.channel;

    const config: DiscordGuildConfig = this.guildConfigs.get(interaction.guildId!) || {
      guildId: interaction.guildId!,
      linkedArtistIds: [],
      enableNowPlaying: true,
      enableRichPresence: true,
    };

    if (announcementsChannel) {
      config.announcementsChannelId = announcementsChannel.id;
    }
    if (nowPlayingChannel) {
      config.nowPlayingChannelId = nowPlayingChannel.id;
    }

    this.guildConfigs.set(interaction.guildId!, config);

    const embed = new EmbedBuilder()
      .setColor('#7C3AED')
      .setTitle('Bot Configuration Updated')
      .addFields(
        {
          name: 'Announcements Channel',
          value: config.announcementsChannelId
            ? `<#${config.announcementsChannelId}>`
            : 'Not set',
          inline: true,
        },
        {
          name: 'Now Playing Channel',
          value: config.nowPlayingChannelId
            ? `<#${config.nowPlayingChannelId}>`
            : 'Not set',
          inline: true,
        }
      );

    await interaction.reply({ embeds: [embed], ephemeral: true });
  }

  // ============================================================================
  // Button Handlers
  // ============================================================================

  private async handleButton(interaction: ButtonInteraction): Promise<void> {
    const [action, id] = interaction.customId.split('_');

    const userLink = this.userLinks.get(interaction.user.id);

    switch (action) {
      case 'play':
        if (!userLink) {
          await interaction.reply({
            content: 'Link your account first with `/link`',
            ephemeral: true,
          });
          return;
        }

        await fetch(`/api/users/${userLink.mycelixUserId}/queue/add`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ trackId: id, playNow: true }),
        });

        await interaction.reply({
          content: 'Track added to your queue!',
          ephemeral: true,
        });
        break;

      case 'follow':
        if (!userLink) {
          await interaction.reply({
            content: 'Link your account first with `/link`',
            ephemeral: true,
          });
          return;
        }

        await fetch(`/api/users/${userLink.mycelixUserId}/following/artists`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ artistId: id }),
        });

        await interaction.reply({
          content: 'Artist followed!',
          ephemeral: true,
        });
        break;

      case 'like':
        if (!userLink) {
          await interaction.reply({
            content: 'Link your account first with `/link`',
            ephemeral: true,
          });
          return;
        }

        await fetch(`/api/users/${userLink.mycelixUserId}/library/tracks`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ trackId: id }),
        });

        await interaction.reply({
          content: 'Track added to your library!',
          ephemeral: true,
        });
        break;
    }
  }

  // ============================================================================
  // Voice State Handling
  // ============================================================================

  private async handleVoiceStateUpdate(oldState: VoiceState, newState: VoiceState): Promise<void> {
    const userLink = this.userLinks.get(newState.member?.id || '');
    if (!userLink) return;

    // Check if user left a listening party voice channel
    const party = Array.from(this.listeningParties.values()).find(
      p => p.voiceChannelId === oldState.channelId && p.isActive
    );

    if (party && !newState.channelId) {
      // User left the party voice channel
      party.participants = party.participants.filter(p => p !== userLink.mycelixUserId);

      // If host left, end the party
      if (party.hostDiscordId === newState.member?.id) {
        party.isActive = false;

        const channel = this.client.channels.cache.get(party.textChannelId) as TextChannel;
        if (channel) {
          await channel.send('The listening party has ended because the host left.');
        }
      }
    }
  }

  // ============================================================================
  // Announcements
  // ============================================================================

  async sendNewReleaseAnnouncement(
    artistId: string,
    release: {
      title: string;
      type: 'single' | 'album' | 'ep';
      artworkUrl: string;
      releaseUrl: string;
      trackCount: number;
    }
  ): Promise<void> {
    // Find guilds that follow this artist
    for (const [guildId, config] of this.guildConfigs) {
      if (
        config.linkedArtistIds.includes(artistId) &&
        config.announcementsChannelId
      ) {
        const channel = this.client.channels.cache.get(
          config.announcementsChannelId
        ) as TextChannel;

        if (!channel) continue;

        const embed = new EmbedBuilder()
          .setColor('#10B981')
          .setTitle(`New ${release.type.toUpperCase()} Release!`)
          .setDescription(`**${release.title}**`)
          .setImage(release.artworkUrl)
          .addFields(
            { name: 'Tracks', value: release.trackCount.toString(), inline: true }
          )
          .setTimestamp();

        const row = new ActionRowBuilder<ButtonBuilder>().addComponents(
          new ButtonBuilder()
            .setLabel('Listen Now')
            .setStyle(ButtonStyle.Link)
            .setURL(release.releaseUrl)
        );

        await channel.send({ embeds: [embed], components: [row] });
      }
    }
  }

  async sendLiveStreamAnnouncement(
    artistId: string,
    stream: {
      title: string;
      artistName: string;
      artworkUrl: string;
      streamUrl: string;
    }
  ): Promise<void> {
    for (const [guildId, config] of this.guildConfigs) {
      if (
        config.linkedArtistIds.includes(artistId) &&
        config.announcementsChannelId
      ) {
        const channel = this.client.channels.cache.get(
          config.announcementsChannelId
        ) as TextChannel;

        if (!channel) continue;

        const embed = new EmbedBuilder()
          .setColor('#EF4444')
          .setTitle('LIVE NOW!')
          .setDescription(`**${stream.artistName}** is streaming: ${stream.title}`)
          .setThumbnail(stream.artworkUrl)
          .setTimestamp();

        const row = new ActionRowBuilder<ButtonBuilder>().addComponents(
          new ButtonBuilder()
            .setLabel('Join Stream')
            .setStyle(ButtonStyle.Link)
            .setURL(stream.streamUrl)
        );

        await channel.send({
          content: '@here',
          embeds: [embed],
          components: [row],
        });
      }
    }
  }

  // ============================================================================
  // Now Playing Updates
  // ============================================================================

  async updateNowPlaying(update: NowPlayingUpdate): Promise<void> {
    // Find Discord user linked to this Mycelix user
    const discordId = Array.from(this.userLinks.entries())
      .find(([_, link]) => link.mycelixUserId === update.userId)?.[0];

    if (!discordId) return;

    const userLink = this.userLinks.get(discordId);
    if (!userLink?.showNowPlaying) return;

    // Update listening party if active
    const party = Array.from(this.listeningParties.values()).find(
      p => p.hostUserId === update.userId && p.isActive
    );

    if (party) {
      party.currentTrackId = update.trackId;

      const channel = this.client.channels.cache.get(party.textChannelId) as TextChannel;
      if (channel) {
        const embed = this.createNowPlayingEmbed(update, 'Party');
        await channel.send({ embeds: [embed] });

        // Sync participants
        for (const participantId of party.participants) {
          if (participantId !== update.userId) {
            await fetch('/api/playback/sync', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({
                userId: participantId,
                trackId: update.trackId,
                position: update.position,
              }),
            });
          }
        }
      }
    }
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private createNowPlayingEmbed(
    track: any,
    username: string
  ): EmbedBuilder {
    return new EmbedBuilder()
      .setColor('#7C3AED')
      .setAuthor({ name: `${username} is listening to` })
      .setTitle(track.title)
      .setDescription(`by **${track.artist}**\non *${track.album}*`)
      .setThumbnail(track.artworkUrl)
      .setFooter({ text: 'Mycelix' });
  }

  private createNowPlayingButtons(trackId: string): ActionRowBuilder<ButtonBuilder> {
    return new ActionRowBuilder<ButtonBuilder>().addComponents(
      new ButtonBuilder()
        .setLabel('Listen')
        .setStyle(ButtonStyle.Link)
        .setURL(`https://mycelix.io/track/${trackId}`),
      new ButtonBuilder()
        .setCustomId(`like_${trackId}`)
        .setLabel('Like')
        .setStyle(ButtonStyle.Secondary)
        .setEmoji('❤️'),
      new ButtonBuilder()
        .setCustomId(`play_${trackId}`)
        .setLabel('Add to Queue')
        .setStyle(ButtonStyle.Primary)
    );
  }

  private formatDuration(ms: number): string {
    const hours = Math.floor(ms / 3600000);
    const minutes = Math.floor((ms % 3600000) / 60000);

    if (hours > 0) {
      return `${hours}h ${minutes}m`;
    }
    return `${minutes}m`;
  }

  private formatNumber(num: number): string {
    if (num >= 1000000) {
      return `${(num / 1000000).toFixed(1)}M`;
    }
    if (num >= 1000) {
      return `${(num / 1000).toFixed(1)}K`;
    }
    return num.toString();
  }
}

export const discordBot = new DiscordBotService();
export default discordBot;
