// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Push Notification Service
 * Handles push notifications for mobile app
 */

import * as Notifications from 'expo-notifications';
import * as Device from 'expo-device';
import { Platform, AppState, AppStateStatus } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import Constants from 'expo-constants';

// Types
export interface NotificationChannel {
  id: string;
  name: string;
  description: string;
  importance: Notifications.AndroidImportance;
  sound: boolean;
  vibration: boolean;
  badge: boolean;
}

export interface NotificationPreferences {
  enabled: boolean;
  channels: {
    newReleases: boolean;
    follows: boolean;
    likes: boolean;
    comments: boolean;
    collaborations: boolean;
    royalties: boolean;
    marketing: boolean;
    system: boolean;
  };
  quietHours: {
    enabled: boolean;
    start: string; // HH:MM
    end: string;   // HH:MM
  };
  sounds: {
    enabled: boolean;
    customSound?: string;
  };
}

export interface PushNotification {
  id: string;
  type: NotificationType;
  title: string;
  body: string;
  data?: Record<string, any>;
  imageUrl?: string;
  deepLink?: string;
  sentAt: Date;
  readAt?: Date;
}

export type NotificationType =
  | 'new_release'
  | 'new_follower'
  | 'track_liked'
  | 'comment'
  | 'collaboration_invite'
  | 'collaboration_update'
  | 'royalty_payment'
  | 'marketing'
  | 'system';

type NotificationListener = (notification: Notifications.Notification) => void;
type ResponseListener = (response: Notifications.NotificationResponse) => void;

const STORAGE_KEYS = {
  PUSH_TOKEN: 'push:token',
  PREFERENCES: 'push:preferences',
  NOTIFICATIONS: 'push:notifications',
};

const DEFAULT_PREFERENCES: NotificationPreferences = {
  enabled: true,
  channels: {
    newReleases: true,
    follows: true,
    likes: true,
    comments: true,
    collaborations: true,
    royalties: true,
    marketing: false,
    system: true,
  },
  quietHours: {
    enabled: false,
    start: '22:00',
    end: '08:00',
  },
  sounds: {
    enabled: true,
  },
};

// Notification channels for Android
const NOTIFICATION_CHANNELS: NotificationChannel[] = [
  {
    id: 'new_releases',
    name: 'New Releases',
    description: 'Notifications for new music from artists you follow',
    importance: Notifications.AndroidImportance.HIGH,
    sound: true,
    vibration: true,
    badge: true,
  },
  {
    id: 'social',
    name: 'Social',
    description: 'Follows, likes, and comments',
    importance: Notifications.AndroidImportance.DEFAULT,
    sound: true,
    vibration: false,
    badge: true,
  },
  {
    id: 'collaborations',
    name: 'Collaborations',
    description: 'Updates on your music collaborations',
    importance: Notifications.AndroidImportance.HIGH,
    sound: true,
    vibration: true,
    badge: true,
  },
  {
    id: 'royalties',
    name: 'Royalties',
    description: 'Payment and earnings notifications',
    importance: Notifications.AndroidImportance.HIGH,
    sound: true,
    vibration: true,
    badge: true,
  },
  {
    id: 'marketing',
    name: 'Marketing',
    description: 'Promotions and recommendations',
    importance: Notifications.AndroidImportance.LOW,
    sound: false,
    vibration: false,
    badge: false,
  },
  {
    id: 'system',
    name: 'System',
    description: 'Important system notifications',
    importance: Notifications.AndroidImportance.MAX,
    sound: true,
    vibration: true,
    badge: true,
  },
];

export class PushNotificationService {
  private static instance: PushNotificationService;
  private pushToken: string | null = null;
  private preferences: NotificationPreferences = DEFAULT_PREFERENCES;
  private notifications: PushNotification[] = [];
  private notificationListeners: Set<NotificationListener> = new Set();
  private responseListeners: Set<ResponseListener> = new Set();
  private notificationSubscription: Notifications.Subscription | null = null;
  private responseSubscription: Notifications.Subscription | null = null;
  private appStateSubscription: any = null;

  private constructor() {
    this.initialize();
  }

  static getInstance(): PushNotificationService {
    if (!PushNotificationService.instance) {
      PushNotificationService.instance = new PushNotificationService();
    }
    return PushNotificationService.instance;
  }

  /**
   * Initialize push notifications
   */
  private async initialize(): Promise<void> {
    // Load persisted data
    await this.loadPersistedData();

    // Configure notification handler
    Notifications.setNotificationHandler({
      handleNotification: async (notification) => {
        const shouldShow = await this.shouldShowNotification(notification);
        return {
          shouldShowAlert: shouldShow,
          shouldPlaySound: shouldShow && this.preferences.sounds.enabled,
          shouldSetBadge: shouldShow,
        };
      },
    });

    // Setup Android notification channels
    if (Platform.OS === 'android') {
      await this.setupAndroidChannels();
    }

    // Setup listeners
    this.setupListeners();

    // Handle app state changes
    this.appStateSubscription = AppState.addEventListener(
      'change',
      this.handleAppStateChange.bind(this)
    );
  }

  /**
   * Register for push notifications
   */
  async registerForPushNotifications(): Promise<string | null> {
    // Check if physical device
    if (!Device.isDevice) {
      console.warn('Push notifications require a physical device');
      return null;
    }

    // Check existing permissions
    const { status: existingStatus } = await Notifications.getPermissionsAsync();
    let finalStatus = existingStatus;

    // Request permissions if not granted
    if (existingStatus !== 'granted') {
      const { status } = await Notifications.requestPermissionsAsync();
      finalStatus = status;
    }

    if (finalStatus !== 'granted') {
      console.warn('Push notification permission not granted');
      return null;
    }

    // Get push token
    try {
      const projectId = Constants.expoConfig?.extra?.eas?.projectId;
      const token = (await Notifications.getExpoPushTokenAsync({
        projectId,
      })).data;

      this.pushToken = token;
      await AsyncStorage.setItem(STORAGE_KEYS.PUSH_TOKEN, token);

      // Register token with backend
      await this.registerTokenWithBackend(token);

      return token;
    } catch (error) {
      console.error('Failed to get push token:', error);
      return null;
    }
  }

  /**
   * Get push token
   */
  getPushToken(): string | null {
    return this.pushToken;
  }

  /**
   * Update notification preferences
   */
  async updatePreferences(preferences: Partial<NotificationPreferences>): Promise<void> {
    this.preferences = { ...this.preferences, ...preferences };
    await AsyncStorage.setItem(STORAGE_KEYS.PREFERENCES, JSON.stringify(this.preferences));

    // Sync with backend
    await this.syncPreferencesWithBackend();
  }

  /**
   * Get notification preferences
   */
  getPreferences(): NotificationPreferences {
    return { ...this.preferences };
  }

  /**
   * Get all notifications
   */
  getNotifications(): PushNotification[] {
    return [...this.notifications];
  }

  /**
   * Get unread notification count
   */
  getUnreadCount(): number {
    return this.notifications.filter(n => !n.readAt).length;
  }

  /**
   * Mark notification as read
   */
  async markAsRead(notificationId: string): Promise<void> {
    const notification = this.notifications.find(n => n.id === notificationId);
    if (notification) {
      notification.readAt = new Date();
      await this.persistNotifications();

      // Update badge
      await this.updateBadge();
    }
  }

  /**
   * Mark all notifications as read
   */
  async markAllAsRead(): Promise<void> {
    const now = new Date();
    this.notifications.forEach(n => {
      if (!n.readAt) n.readAt = now;
    });
    await this.persistNotifications();
    await this.updateBadge();
  }

  /**
   * Clear all notifications
   */
  async clearAll(): Promise<void> {
    this.notifications = [];
    await this.persistNotifications();
    await Notifications.setBadgeCountAsync(0);
  }

  /**
   * Delete notification
   */
  async deleteNotification(notificationId: string): Promise<void> {
    this.notifications = this.notifications.filter(n => n.id !== notificationId);
    await this.persistNotifications();
    await this.updateBadge();
  }

  /**
   * Schedule local notification
   */
  async scheduleLocalNotification(
    title: string,
    body: string,
    options: {
      data?: Record<string, any>;
      trigger?: Notifications.NotificationTriggerInput;
      channelId?: string;
    } = {}
  ): Promise<string> {
    const identifier = await Notifications.scheduleNotificationAsync({
      content: {
        title,
        body,
        data: options.data,
        sound: this.preferences.sounds.enabled ? 'default' : undefined,
        ...(Platform.OS === 'android' && { channelId: options.channelId || 'system' }),
      },
      trigger: options.trigger || null,
    });

    return identifier;
  }

  /**
   * Cancel scheduled notification
   */
  async cancelScheduledNotification(identifier: string): Promise<void> {
    await Notifications.cancelScheduledNotificationAsync(identifier);
  }

  /**
   * Cancel all scheduled notifications
   */
  async cancelAllScheduledNotifications(): Promise<void> {
    await Notifications.cancelAllScheduledNotificationsAsync();
  }

  /**
   * Add notification listener
   */
  addNotificationListener(listener: NotificationListener): () => void {
    this.notificationListeners.add(listener);
    return () => this.notificationListeners.delete(listener);
  }

  /**
   * Add notification response listener
   */
  addResponseListener(listener: ResponseListener): () => void {
    this.responseListeners.add(listener);
    return () => this.responseListeners.delete(listener);
  }

  /**
   * Handle deep link from notification
   */
  handleDeepLink(data: Record<string, any>): { screen: string; params: Record<string, any> } | null {
    const { type, ...params } = data;

    switch (type) {
      case 'new_release':
        return { screen: 'Track', params: { trackId: params.trackId } };

      case 'new_follower':
        return { screen: 'Profile', params: { userId: params.userId } };

      case 'track_liked':
      case 'comment':
        return { screen: 'Track', params: { trackId: params.trackId } };

      case 'collaboration_invite':
      case 'collaboration_update':
        return { screen: 'Collaboration', params: { collaborationId: params.collaborationId } };

      case 'royalty_payment':
        return { screen: 'Earnings', params: {} };

      default:
        return null;
    }
  }

  /**
   * Cleanup
   */
  destroy(): void {
    this.notificationSubscription?.remove();
    this.responseSubscription?.remove();
    this.appStateSubscription?.remove();
  }

  // Private methods

  private async loadPersistedData(): Promise<void> {
    try {
      // Load token
      const token = await AsyncStorage.getItem(STORAGE_KEYS.PUSH_TOKEN);
      if (token) this.pushToken = token;

      // Load preferences
      const prefsJson = await AsyncStorage.getItem(STORAGE_KEYS.PREFERENCES);
      if (prefsJson) {
        this.preferences = { ...DEFAULT_PREFERENCES, ...JSON.parse(prefsJson) };
      }

      // Load notifications
      const notificationsJson = await AsyncStorage.getItem(STORAGE_KEYS.NOTIFICATIONS);
      if (notificationsJson) {
        this.notifications = JSON.parse(notificationsJson).map((n: any) => ({
          ...n,
          sentAt: new Date(n.sentAt),
          readAt: n.readAt ? new Date(n.readAt) : undefined,
        }));
      }
    } catch (error) {
      console.error('Failed to load push notification data:', error);
    }
  }

  private async persistNotifications(): Promise<void> {
    try {
      // Keep only last 100 notifications
      const toSave = this.notifications.slice(0, 100);
      await AsyncStorage.setItem(STORAGE_KEYS.NOTIFICATIONS, JSON.stringify(toSave));
    } catch (error) {
      console.error('Failed to persist notifications:', error);
    }
  }

  private async setupAndroidChannels(): Promise<void> {
    for (const channel of NOTIFICATION_CHANNELS) {
      await Notifications.setNotificationChannelAsync(channel.id, {
        name: channel.name,
        description: channel.description,
        importance: channel.importance,
        sound: channel.sound ? 'default' : undefined,
        vibrationPattern: channel.vibration ? [0, 250, 250, 250] : undefined,
        enableVibrate: channel.vibration,
      });
    }
  }

  private setupListeners(): void {
    // Listen for notifications while app is foregrounded
    this.notificationSubscription = Notifications.addNotificationReceivedListener(
      (notification) => {
        this.handleNotificationReceived(notification);
      }
    );

    // Listen for notification interactions
    this.responseSubscription = Notifications.addNotificationResponseReceivedListener(
      (response) => {
        this.handleNotificationResponse(response);
      }
    );
  }

  private handleNotificationReceived(notification: Notifications.Notification): void {
    // Add to local notifications
    const pushNotification: PushNotification = {
      id: notification.request.identifier,
      type: notification.request.content.data?.type as NotificationType || 'system',
      title: notification.request.content.title || '',
      body: notification.request.content.body || '',
      data: notification.request.content.data,
      sentAt: new Date(),
    };

    this.notifications.unshift(pushNotification);
    this.persistNotifications();

    // Notify listeners
    this.notificationListeners.forEach(listener => listener(notification));
  }

  private handleNotificationResponse(response: Notifications.NotificationResponse): void {
    // Mark as read
    this.markAsRead(response.notification.request.identifier);

    // Notify listeners
    this.responseListeners.forEach(listener => listener(response));
  }

  private async shouldShowNotification(notification: Notifications.Notification): Promise<boolean> {
    if (!this.preferences.enabled) return false;

    const type = notification.request.content.data?.type as NotificationType;

    // Check channel preferences
    const channelMap: Record<NotificationType, keyof typeof this.preferences.channels> = {
      new_release: 'newReleases',
      new_follower: 'follows',
      track_liked: 'likes',
      comment: 'comments',
      collaboration_invite: 'collaborations',
      collaboration_update: 'collaborations',
      royalty_payment: 'royalties',
      marketing: 'marketing',
      system: 'system',
    };

    const channel = channelMap[type];
    if (channel && !this.preferences.channels[channel]) return false;

    // Check quiet hours
    if (this.preferences.quietHours.enabled) {
      const now = new Date();
      const currentTime = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`;
      const { start, end } = this.preferences.quietHours;

      if (start < end) {
        // Same day quiet hours (e.g., 09:00 - 17:00)
        if (currentTime >= start && currentTime <= end) return false;
      } else {
        // Overnight quiet hours (e.g., 22:00 - 08:00)
        if (currentTime >= start || currentTime <= end) return false;
      }
    }

    return true;
  }

  private async updateBadge(): Promise<void> {
    const count = this.getUnreadCount();
    await Notifications.setBadgeCountAsync(count);
  }

  private handleAppStateChange(state: AppStateStatus): void {
    if (state === 'active') {
      // Refresh notifications when app becomes active
      this.updateBadge();
    }
  }

  private async registerTokenWithBackend(token: string): Promise<void> {
    // This would call your API to register the token
    console.log('Registering push token with backend:', token);
  }

  private async syncPreferencesWithBackend(): Promise<void> {
    // This would sync preferences with your API
    console.log('Syncing notification preferences with backend');
  }
}

export default PushNotificationService;
