// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Push Notifications Service
 *
 * Handles push notification registration, handling, and deep linking
 * for the Mycelix mobile app.
 */

import messaging from '@react-native-firebase/messaging';
import notifee, {
  AndroidImportance,
  AndroidStyle,
  EventType,
  Notification,
  Event,
} from '@notifee/react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { Linking, Platform } from 'react-native';

// ============================================================================
// Types
// ============================================================================

export interface NotificationPreferences {
  newReleases: boolean;
  artistUpdates: boolean;
  circleInvites: boolean;
  socialActivity: boolean;
  patronUpdates: boolean;
  systemNotifications: boolean;
  marketplaceActivity: boolean;
  liveEvents: boolean;
  dailyMix: boolean;
  weeklyRecap: boolean;
}

export interface PushNotificationPayload {
  type: NotificationType;
  title: string;
  body: string;
  imageUrl?: string;
  data: Record<string, string>;
  actions?: NotificationAction[];
}

export interface NotificationAction {
  id: string;
  title: string;
  icon?: string;
}

export type NotificationType =
  | 'new_release'
  | 'artist_update'
  | 'circle_invite'
  | 'circle_activity'
  | 'follow'
  | 'like'
  | 'comment'
  | 'mention'
  | 'patron_content'
  | 'patron_milestone'
  | 'marketplace_sale'
  | 'marketplace_offer'
  | 'live_started'
  | 'live_reminder'
  | 'daily_mix'
  | 'weekly_recap'
  | 'achievement'
  | 'system';

type NotificationHandler = (payload: PushNotificationPayload) => void | Promise<void>;
type DeepLinkHandler = (url: string) => void;

// ============================================================================
// Constants
// ============================================================================

const STORAGE_KEYS = {
  FCM_TOKEN: 'push:fcmToken',
  PREFERENCES: 'push:preferences',
  NOTIFICATION_HISTORY: 'push:history',
};

const CHANNEL_IDS = {
  DEFAULT: 'default',
  MUSIC: 'music',
  SOCIAL: 'social',
  LIVE: 'live',
  MARKETING: 'marketing',
};

// ============================================================================
// Push Notifications Service
// ============================================================================

class PushNotificationsService {
  private fcmToken: string | null = null;
  private preferences: NotificationPreferences;
  private handlers: Map<NotificationType, NotificationHandler[]> = new Map();
  private deepLinkHandler: DeepLinkHandler | null = null;
  private isInitialized = false;

  constructor() {
    this.preferences = this.getDefaultPreferences();
  }

  // ============================================================================
  // Initialization
  // ============================================================================

  async initialize(): Promise<void> {
    if (this.isInitialized) return;

    // Request permission
    const authStatus = await messaging().requestPermission();
    const enabled =
      authStatus === messaging.AuthorizationStatus.AUTHORIZED ||
      authStatus === messaging.AuthorizationStatus.PROVISIONAL;

    if (!enabled) {
      console.log('Push notification permission denied');
      return;
    }

    // Create notification channels (Android)
    await this.createNotificationChannels();

    // Get FCM token
    this.fcmToken = await messaging().getToken();
    await this.saveFcmToken(this.fcmToken);

    // Listen for token refresh
    messaging().onTokenRefresh(async (token) => {
      this.fcmToken = token;
      await this.saveFcmToken(token);
      await this.syncTokenWithServer(token);
    });

    // Handle foreground messages
    messaging().onMessage(async (remoteMessage) => {
      await this.handleForegroundMessage(remoteMessage);
    });

    // Handle background/quit message opens
    messaging().onNotificationOpenedApp((remoteMessage) => {
      this.handleNotificationOpen(remoteMessage);
    });

    // Check if app was opened from notification
    const initialMessage = await messaging().getInitialNotification();
    if (initialMessage) {
      this.handleNotificationOpen(initialMessage);
    }

    // Handle notifee events (for local notifications)
    notifee.onForegroundEvent(this.handleNotifeeEvent.bind(this));
    notifee.onBackgroundEvent(this.handleNotifeeEvent.bind(this));

    // Load preferences
    await this.loadPreferences();

    this.isInitialized = true;
  }

  private async createNotificationChannels(): Promise<void> {
    if (Platform.OS !== 'android') return;

    await notifee.createChannel({
      id: CHANNEL_IDS.DEFAULT,
      name: 'Default',
      importance: AndroidImportance.DEFAULT,
    });

    await notifee.createChannel({
      id: CHANNEL_IDS.MUSIC,
      name: 'New Music',
      description: 'New releases from artists you follow',
      importance: AndroidImportance.HIGH,
      sound: 'notification_music',
    });

    await notifee.createChannel({
      id: CHANNEL_IDS.SOCIAL,
      name: 'Social',
      description: 'Likes, comments, follows, and mentions',
      importance: AndroidImportance.DEFAULT,
    });

    await notifee.createChannel({
      id: CHANNEL_IDS.LIVE,
      name: 'Live Events',
      description: 'Live streams and listening circles',
      importance: AndroidImportance.HIGH,
      sound: 'notification_live',
    });

    await notifee.createChannel({
      id: CHANNEL_IDS.MARKETING,
      name: 'Updates & Recommendations',
      description: 'Daily mixes, weekly recaps, and personalized recommendations',
      importance: AndroidImportance.LOW,
    });
  }

  // ============================================================================
  // Message Handling
  // ============================================================================

  private async handleForegroundMessage(
    remoteMessage: any
  ): Promise<void> {
    const payload = this.parsePayload(remoteMessage);

    // Check if notification type is enabled
    if (!this.isNotificationTypeEnabled(payload.type)) {
      return;
    }

    // Show local notification
    await this.displayNotification(payload);

    // Call registered handlers
    const handlers = this.handlers.get(payload.type) || [];
    for (const handler of handlers) {
      await handler(payload);
    }
  }

  private handleNotificationOpen(remoteMessage: any): void {
    const payload = this.parsePayload(remoteMessage);
    const deepLink = this.buildDeepLink(payload);

    if (deepLink && this.deepLinkHandler) {
      this.deepLinkHandler(deepLink);
    }
  }

  private async handleNotifeeEvent(event: Event): Promise<void> {
    const { type, detail } = event;

    if (type === EventType.PRESS) {
      const payload = detail.notification?.data as PushNotificationPayload;
      if (payload) {
        const deepLink = this.buildDeepLink(payload);
        if (deepLink && this.deepLinkHandler) {
          this.deepLinkHandler(deepLink);
        }
      }
    }

    if (type === EventType.ACTION_PRESS) {
      const actionId = detail.pressAction?.id;
      const payload = detail.notification?.data as PushNotificationPayload;

      if (actionId && payload) {
        await this.handleNotificationAction(actionId, payload);
      }
    }
  }

  private async handleNotificationAction(
    actionId: string,
    payload: PushNotificationPayload
  ): Promise<void> {
    switch (actionId) {
      case 'play':
        // Play the track/album mentioned in notification
        const trackId = payload.data.trackId;
        if (trackId) {
          // Would trigger playback
        }
        break;

      case 'join_circle':
        const circleId = payload.data.circleId;
        if (circleId && this.deepLinkHandler) {
          this.deepLinkHandler(`mycelix://circle/${circleId}/join`);
        }
        break;

      case 'view':
        const deepLink = this.buildDeepLink(payload);
        if (deepLink && this.deepLinkHandler) {
          this.deepLinkHandler(deepLink);
        }
        break;

      case 'dismiss':
        // Just dismiss, no action needed
        break;
    }
  }

  // ============================================================================
  // Display Notifications
  // ============================================================================

  private async displayNotification(payload: PushNotificationPayload): Promise<void> {
    const channelId = this.getChannelForType(payload.type);

    const notification: Notification = {
      title: payload.title,
      body: payload.body,
      data: payload.data,
      android: {
        channelId,
        pressAction: { id: 'default' },
        actions: payload.actions?.map(action => ({
          title: action.title,
          pressAction: { id: action.id },
        })),
      },
      ios: {
        categoryId: payload.type,
      },
    };

    // Add big picture style for image notifications
    if (payload.imageUrl) {
      notification.android = {
        ...notification.android,
        style: {
          type: AndroidStyle.BIGPICTURE,
          picture: payload.imageUrl,
        },
      };
    }

    await notifee.displayNotification(notification);

    // Store in history
    await this.addToHistory(payload);
  }

  async displayLocalNotification(
    title: string,
    body: string,
    data?: Record<string, string>
  ): Promise<void> {
    await notifee.displayNotification({
      title,
      body,
      data,
      android: {
        channelId: CHANNEL_IDS.DEFAULT,
      },
    });
  }

  // ============================================================================
  // Handlers
  // ============================================================================

  onNotification(
    type: NotificationType,
    handler: NotificationHandler
  ): () => void {
    const handlers = this.handlers.get(type) || [];
    handlers.push(handler);
    this.handlers.set(type, handlers);

    return () => {
      const currentHandlers = this.handlers.get(type) || [];
      this.handlers.set(
        type,
        currentHandlers.filter(h => h !== handler)
      );
    };
  }

  setDeepLinkHandler(handler: DeepLinkHandler): void {
    this.deepLinkHandler = handler;
  }

  // ============================================================================
  // Preferences
  // ============================================================================

  async updatePreferences(updates: Partial<NotificationPreferences>): Promise<void> {
    this.preferences = { ...this.preferences, ...updates };
    await this.savePreferences();
    await this.syncPreferencesWithServer();
  }

  getPreferences(): NotificationPreferences {
    return { ...this.preferences };
  }

  private isNotificationTypeEnabled(type: NotificationType): boolean {
    const typeToPreference: Record<NotificationType, keyof NotificationPreferences> = {
      new_release: 'newReleases',
      artist_update: 'artistUpdates',
      circle_invite: 'circleInvites',
      circle_activity: 'circleInvites',
      follow: 'socialActivity',
      like: 'socialActivity',
      comment: 'socialActivity',
      mention: 'socialActivity',
      patron_content: 'patronUpdates',
      patron_milestone: 'patronUpdates',
      marketplace_sale: 'marketplaceActivity',
      marketplace_offer: 'marketplaceActivity',
      live_started: 'liveEvents',
      live_reminder: 'liveEvents',
      daily_mix: 'dailyMix',
      weekly_recap: 'weeklyRecap',
      achievement: 'socialActivity',
      system: 'systemNotifications',
    };

    const preferenceKey = typeToPreference[type];
    return preferenceKey ? this.preferences[preferenceKey] : true;
  }

  private getDefaultPreferences(): NotificationPreferences {
    return {
      newReleases: true,
      artistUpdates: true,
      circleInvites: true,
      socialActivity: true,
      patronUpdates: true,
      systemNotifications: true,
      marketplaceActivity: true,
      liveEvents: true,
      dailyMix: false,
      weeklyRecap: true,
    };
  }

  // ============================================================================
  // Token Management
  // ============================================================================

  getFcmToken(): string | null {
    return this.fcmToken;
  }

  private async saveFcmToken(token: string): Promise<void> {
    await AsyncStorage.setItem(STORAGE_KEYS.FCM_TOKEN, token);
  }

  private async syncTokenWithServer(token: string): Promise<void> {
    // Would call API to update FCM token
    try {
      await fetch('/api/users/push-token', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ token, platform: Platform.OS }),
      });
    } catch (error) {
      console.error('Failed to sync FCM token:', error);
    }
  }

  private async syncPreferencesWithServer(): Promise<void> {
    try {
      await fetch('/api/users/notification-preferences', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(this.preferences),
      });
    } catch (error) {
      console.error('Failed to sync preferences:', error);
    }
  }

  // ============================================================================
  // History
  // ============================================================================

  async getNotificationHistory(limit = 50): Promise<PushNotificationPayload[]> {
    const json = await AsyncStorage.getItem(STORAGE_KEYS.NOTIFICATION_HISTORY);
    if (!json) return [];

    const history = JSON.parse(json) as PushNotificationPayload[];
    return history.slice(0, limit);
  }

  private async addToHistory(payload: PushNotificationPayload): Promise<void> {
    const history = await this.getNotificationHistory(100);
    history.unshift(payload);

    // Keep only last 100 notifications
    await AsyncStorage.setItem(
      STORAGE_KEYS.NOTIFICATION_HISTORY,
      JSON.stringify(history.slice(0, 100))
    );
  }

  async clearNotificationHistory(): Promise<void> {
    await AsyncStorage.removeItem(STORAGE_KEYS.NOTIFICATION_HISTORY);
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private parsePayload(remoteMessage: any): PushNotificationPayload {
    return {
      type: remoteMessage.data?.type || 'system',
      title: remoteMessage.notification?.title || '',
      body: remoteMessage.notification?.body || '',
      imageUrl: remoteMessage.notification?.imageUrl,
      data: remoteMessage.data || {},
      actions: remoteMessage.data?.actions
        ? JSON.parse(remoteMessage.data.actions)
        : undefined,
    };
  }

  private buildDeepLink(payload: PushNotificationPayload): string | null {
    const { type, data } = payload;

    switch (type) {
      case 'new_release':
        return data.albumId
          ? `mycelix://album/${data.albumId}`
          : data.trackId
            ? `mycelix://track/${data.trackId}`
            : null;

      case 'artist_update':
        return data.artistId ? `mycelix://artist/${data.artistId}` : null;

      case 'circle_invite':
      case 'circle_activity':
        return data.circleId ? `mycelix://circle/${data.circleId}` : null;

      case 'follow':
      case 'mention':
        return data.userId ? `mycelix://profile/${data.userId}` : null;

      case 'like':
      case 'comment':
        return data.trackId ? `mycelix://track/${data.trackId}` : null;

      case 'patron_content':
        return data.contentId
          ? `mycelix://patron-content/${data.contentId}`
          : null;

      case 'marketplace_sale':
      case 'marketplace_offer':
        return data.listingId
          ? `mycelix://marketplace/${data.listingId}`
          : null;

      case 'live_started':
      case 'live_reminder':
        return data.liveId ? `mycelix://live/${data.liveId}` : null;

      case 'daily_mix':
        return 'mycelix://daily-mix';

      case 'weekly_recap':
        return 'mycelix://weekly-recap';

      case 'achievement':
        return data.achievementId
          ? `mycelix://achievements/${data.achievementId}`
          : 'mycelix://achievements';

      default:
        return data.deepLink || null;
    }
  }

  private getChannelForType(type: NotificationType): string {
    switch (type) {
      case 'new_release':
      case 'daily_mix':
        return CHANNEL_IDS.MUSIC;

      case 'follow':
      case 'like':
      case 'comment':
      case 'mention':
      case 'achievement':
        return CHANNEL_IDS.SOCIAL;

      case 'live_started':
      case 'live_reminder':
      case 'circle_invite':
      case 'circle_activity':
        return CHANNEL_IDS.LIVE;

      case 'weekly_recap':
        return CHANNEL_IDS.MARKETING;

      default:
        return CHANNEL_IDS.DEFAULT;
    }
  }

  private async loadPreferences(): Promise<void> {
    const json = await AsyncStorage.getItem(STORAGE_KEYS.PREFERENCES);
    if (json) {
      this.preferences = { ...this.getDefaultPreferences(), ...JSON.parse(json) };
    }
  }

  private async savePreferences(): Promise<void> {
    await AsyncStorage.setItem(STORAGE_KEYS.PREFERENCES, JSON.stringify(this.preferences));
  }
}

export const pushNotifications = new PushNotificationsService();
export default pushNotifications;
