// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { prisma } from '../utils/prisma';
import { encrypt, decrypt } from '../utils/encryption';
import { AppError } from '../middleware/errorHandler';

interface CreateAccountData {
  email: string;
  provider: string;
  imapHost: string;
  imapPort: number;
  imapSecure: boolean;
  imapUser: string;
  imapPassword: string;
  smtpHost: string;
  smtpPort: number;
  smtpSecure: boolean;
  smtpUser: string;
  smtpPassword: string;
}

export class AccountService {
  async createAccount(userId: string, data: CreateAccountData) {
    // Encrypt passwords
    const encryptedImapPassword = encrypt(data.imapPassword);
    const encryptedSmtpPassword = encrypt(data.smtpPassword);

    // If this is the first account, make it default
    const accountCount = await prisma.emailAccount.count({
      where: { userId },
    });

    const account = await prisma.emailAccount.create({
      data: {
        userId,
        email: data.email,
        provider: data.provider,
        imapHost: data.imapHost,
        imapPort: data.imapPort,
        imapSecure: data.imapSecure,
        imapUser: data.imapUser,
        imapPassword: encryptedImapPassword,
        smtpHost: data.smtpHost,
        smtpPort: data.smtpPort,
        smtpSecure: data.smtpSecure,
        smtpUser: data.smtpUser,
        smtpPassword: encryptedSmtpPassword,
        isDefault: accountCount === 0,
      },
    });

    // Create default folders
    await this.createDefaultFolders(userId, account.id);

    // Return account without sensitive data
    const { imapPassword, smtpPassword, ...accountWithoutPasswords } = account;

    return accountWithoutPasswords;
  }

  private async createDefaultFolders(userId: string, accountId: string) {
    const defaultFolders = [
      { name: 'Inbox', path: 'INBOX', type: 'INBOX' as const },
      { name: 'Sent', path: 'INBOX/Sent', type: 'SENT' as const },
      { name: 'Drafts', path: 'INBOX/Drafts', type: 'DRAFTS' as const },
      { name: 'Trash', path: 'INBOX/Trash', type: 'TRASH' as const },
      { name: 'Spam', path: 'INBOX/Spam', type: 'SPAM' as const },
    ];

    for (const folder of defaultFolders) {
      await prisma.folder.create({
        data: {
          userId,
          emailAccountId: accountId,
          ...folder,
        },
      });
    }
  }

  async getAccounts(userId: string) {
    const accounts = await prisma.emailAccount.findMany({
      where: { userId },
      select: {
        id: true,
        email: true,
        provider: true,
        imapHost: true,
        imapPort: true,
        imapSecure: true,
        smtpHost: true,
        smtpPort: true,
        smtpSecure: true,
        isDefault: true,
        lastSyncedAt: true,
        createdAt: true,
        updatedAt: true,
      },
    });

    return accounts;
  }

  async getAccount(userId: string, accountId: string) {
    const account = await prisma.emailAccount.findFirst({
      where: { id: accountId, userId },
      select: {
        id: true,
        email: true,
        provider: true,
        imapHost: true,
        imapPort: true,
        imapSecure: true,
        smtpHost: true,
        smtpPort: true,
        smtpSecure: true,
        isDefault: true,
        lastSyncedAt: true,
        createdAt: true,
        updatedAt: true,
      },
    });

    if (!account) {
      throw new AppError('Account not found', 404);
    }

    return account;
  }

  async updateAccount(userId: string, accountId: string, data: Partial<CreateAccountData>) {
    const account = await prisma.emailAccount.findFirst({
      where: { id: accountId, userId },
    });

    if (!account) {
      throw new AppError('Account not found', 404);
    }

    const updateData: any = { ...data };

    // Encrypt passwords if provided
    if (data.imapPassword) {
      updateData.imapPassword = encrypt(data.imapPassword);
    }
    if (data.smtpPassword) {
      updateData.smtpPassword = encrypt(data.smtpPassword);
    }

    const updatedAccount = await prisma.emailAccount.update({
      where: { id: accountId },
      data: updateData,
      select: {
        id: true,
        email: true,
        provider: true,
        imapHost: true,
        imapPort: true,
        imapSecure: true,
        smtpHost: true,
        smtpPort: true,
        smtpSecure: true,
        isDefault: true,
        lastSyncedAt: true,
        updatedAt: true,
      },
    });

    return updatedAccount;
  }

  async deleteAccount(userId: string, accountId: string) {
    const account = await prisma.emailAccount.findFirst({
      where: { id: accountId, userId },
    });

    if (!account) {
      throw new AppError('Account not found', 404);
    }

    await prisma.emailAccount.delete({
      where: { id: accountId },
    });

    return { message: 'Account deleted successfully' };
  }

  async setDefaultAccount(userId: string, accountId: string) {
    // Remove default from all accounts
    await prisma.emailAccount.updateMany({
      where: { userId },
      data: { isDefault: false },
    });

    // Set new default
    const account = await prisma.emailAccount.update({
      where: { id: accountId },
      data: { isDefault: true },
      select: {
        id: true,
        email: true,
        isDefault: true,
      },
    });

    return account;
  }
}

export const accountService = new AccountService();
