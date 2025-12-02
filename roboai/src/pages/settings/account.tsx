/**
 * Account Settings Page
 *
 * User account settings for managing linked accounts and security.
 * Implements T088-T095 from the task list.
 *
 * Features:
 * - View linked OAuth accounts (GitHub, Google)
 * - Link new accounts
 * - Unlink accounts
 * - Change password
 */
import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useSession } from "@/lib/auth";
import authClient from "@/lib/auth";
import { RequireAuth } from "@/components/Auth";
import "./account.css";

interface LinkedAccount {
  provider: string;
  providerId: string;
  accountId: string;
}

function AccountSettingsContent() {
  const { data: session } = useSession();
  const [linkedAccounts, setLinkedAccounts] = useState<LinkedAccount[]>([]);
  const [isLoadingAccounts, setIsLoadingAccounts] = useState(true);
  const [actionLoading, setActionLoading] = useState<string | null>(null);
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");

  // Password change state
  const [currentPassword, setCurrentPassword] = useState("");
  const [newPassword, setNewPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [passwordError, setPasswordError] = useState("");
  const [passwordSuccess, setPasswordSuccess] = useState("");
  const [isChangingPassword, setIsChangingPassword] = useState(false);

  // Fetch linked accounts on mount
  useEffect(() => {
    fetchLinkedAccounts();
  }, []);

  /**
   * Fetch linked OAuth accounts
   */
  const fetchLinkedAccounts = async () => {
    setIsLoadingAccounts(true);
    try {
      if ("listAccounts" in authClient) {
        const result = await (authClient as any).listAccounts();
        setLinkedAccounts(result?.data || []);
      }
    } catch (err) {
      console.error("[AccountSettings] Failed to fetch accounts:", err);
    } finally {
      setIsLoadingAccounts(false);
    }
  };

  /**
   * Link a new OAuth account
   */
  const handleLinkAccount = async (provider: "github" | "google") => {
    setError("");
    setSuccess("");
    setActionLoading(provider);

    try {
      if ("linkSocial" in authClient) {
        await (authClient as any).linkSocial({
          provider,
          callbackURL: window.location.href,
        });
        // The user will be redirected to OAuth provider
      } else {
        setError("Account linking is not configured.");
      }
    } catch (err: any) {
      console.error(`[AccountSettings] Failed to link ${provider}:`, err);
      setError(err?.message || `Failed to link ${provider} account.`);
    } finally {
      setActionLoading(null);
    }
  };

  /**
   * Unlink an OAuth account
   */
  const handleUnlinkAccount = async (providerId: string) => {
    setError("");
    setSuccess("");
    setActionLoading(providerId);

    try {
      if ("unlinkAccount" in authClient) {
        await (authClient as any).unlinkAccount({
          providerId,
        });
        setSuccess("Account unlinked successfully!");
        // Refresh linked accounts
        await fetchLinkedAccounts();
      } else {
        setError("Account unlinking is not configured.");
      }
    } catch (err: any) {
      console.error("[AccountSettings] Failed to unlink account:", err);
      setError(err?.message || "Failed to unlink account.");
    } finally {
      setActionLoading(null);
    }
  };

  /**
   * Handle password change
   */
  const handleChangePassword = async (e: React.FormEvent) => {
    e.preventDefault();
    setPasswordError("");
    setPasswordSuccess("");

    // Validate
    if (!currentPassword) {
      setPasswordError("Current password is required");
      return;
    }
    if (!newPassword || newPassword.length < 8) {
      setPasswordError("New password must be at least 8 characters");
      return;
    }
    if (newPassword !== confirmPassword) {
      setPasswordError("Passwords do not match");
      return;
    }

    setIsChangingPassword(true);

    try {
      if ("changePassword" in authClient) {
        await (authClient as any).changePassword({
          currentPassword,
          newPassword,
        });
        setPasswordSuccess("Password changed successfully!");
        // Clear form
        setCurrentPassword("");
        setNewPassword("");
        setConfirmPassword("");
      } else {
        setPasswordError("Password change is not configured.");
      }
    } catch (err: any) {
      console.error("[AccountSettings] Password change failed:", err);
      const message = err?.message?.toLowerCase() || "";
      if (message.includes("incorrect") || message.includes("invalid")) {
        setPasswordError("Current password is incorrect");
      } else {
        setPasswordError(err?.message || "Failed to change password.");
      }
    } finally {
      setIsChangingPassword(false);
    }
  };

  /**
   * Check if a provider is linked
   */
  const isProviderLinked = (provider: string) => {
    return linkedAccounts.some(
      (acc) => acc.provider.toLowerCase() === provider.toLowerCase()
    );
  };

  /**
   * Get provider ID for unlinking
   */
  const getProviderId = (provider: string) => {
    const account = linkedAccounts.find(
      (acc) => acc.provider.toLowerCase() === provider.toLowerCase()
    );
    return account?.providerId || account?.accountId;
  };

  if (!session) {
    return null;
  }

  return (
    <div className="account-settings-page">
      <div className="account-settings-card">
        <h1 className="account-settings-title">Account Settings</h1>

        {/* General Messages */}
        {success && (
          <div className="account-message account-message--success" role="status">
            {success}
          </div>
        )}
        {error && (
          <div className="account-message account-message--error" role="alert">
            {error}
          </div>
        )}

        {/* Linked Accounts Section */}
        <section className="account-section">
          <h2 className="account-section-title">Linked Accounts</h2>
          <p className="account-section-desc">
            Connect your social accounts for easier sign-in.
          </p>

          {isLoadingAccounts ? (
            <div className="account-loading">Loading accounts...</div>
          ) : (
            <div className="account-providers">
              {/* GitHub */}
              <div className="account-provider">
                <div className="account-provider-info">
                  <GitHubIcon className="account-provider-icon" />
                  <span className="account-provider-name">GitHub</span>
                  {isProviderLinked("github") && (
                    <span className="account-badge account-badge--linked">
                      Linked
                    </span>
                  )}
                </div>
                {isProviderLinked("github") ? (
                  <button
                    type="button"
                    className="account-btn account-btn--danger"
                    onClick={() => handleUnlinkAccount(getProviderId("github")!)}
                    disabled={actionLoading === getProviderId("github")}
                  >
                    {actionLoading === getProviderId("github")
                      ? "Unlinking..."
                      : "Unlink"}
                  </button>
                ) : (
                  <button
                    type="button"
                    className="account-btn account-btn--secondary"
                    onClick={() => handleLinkAccount("github")}
                    disabled={actionLoading === "github"}
                  >
                    {actionLoading === "github" ? "Linking..." : "Link"}
                  </button>
                )}
              </div>

              {/* Google */}
              <div className="account-provider">
                <div className="account-provider-info">
                  <GoogleIcon className="account-provider-icon" />
                  <span className="account-provider-name">Google</span>
                  {isProviderLinked("google") && (
                    <span className="account-badge account-badge--linked">
                      Linked
                    </span>
                  )}
                </div>
                {isProviderLinked("google") ? (
                  <button
                    type="button"
                    className="account-btn account-btn--danger"
                    onClick={() => handleUnlinkAccount(getProviderId("google")!)}
                    disabled={actionLoading === getProviderId("google")}
                  >
                    {actionLoading === getProviderId("google")
                      ? "Unlinking..."
                      : "Unlink"}
                  </button>
                ) : (
                  <button
                    type="button"
                    className="account-btn account-btn--secondary"
                    onClick={() => handleLinkAccount("google")}
                    disabled={actionLoading === "google"}
                  >
                    {actionLoading === "google" ? "Linking..." : "Link"}
                  </button>
                )}
              </div>
            </div>
          )}
        </section>

        {/* Change Password Section */}
        <section className="account-section">
          <h2 className="account-section-title">Change Password</h2>
          <p className="account-section-desc">
            Update your password to keep your account secure.
          </p>

          <form onSubmit={handleChangePassword} className="account-form">
            {passwordSuccess && (
              <div className="account-message account-message--success" role="status">
                {passwordSuccess}
              </div>
            )}
            {passwordError && (
              <div className="account-message account-message--error" role="alert">
                {passwordError}
              </div>
            )}

            <div className="account-field">
              <label htmlFor="currentPassword" className="account-label">
                Current Password
              </label>
              <input
                id="currentPassword"
                type="password"
                value={currentPassword}
                onChange={(e) => setCurrentPassword(e.target.value)}
                className="account-input"
                placeholder="Enter current password"
                disabled={isChangingPassword}
                autoComplete="current-password"
              />
            </div>

            <div className="account-field">
              <label htmlFor="newPassword" className="account-label">
                New Password
              </label>
              <input
                id="newPassword"
                type="password"
                value={newPassword}
                onChange={(e) => setNewPassword(e.target.value)}
                className="account-input"
                placeholder="At least 8 characters"
                disabled={isChangingPassword}
                autoComplete="new-password"
              />
            </div>

            <div className="account-field">
              <label htmlFor="confirmPassword" className="account-label">
                Confirm New Password
              </label>
              <input
                id="confirmPassword"
                type="password"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                className="account-input"
                placeholder="Confirm new password"
                disabled={isChangingPassword}
                autoComplete="new-password"
              />
            </div>

            <button
              type="submit"
              className="account-btn account-btn--primary"
              disabled={isChangingPassword}
            >
              {isChangingPassword ? "Changing..." : "Change Password"}
            </button>
          </form>
        </section>

        {/* Back to Profile */}
        <div className="account-links">
          <a href="/profile" className="account-link">
            ← Back to Profile
          </a>
        </div>
      </div>
    </div>
  );
}

/**
 * GitHub Icon
 */
function GitHubIcon({ className }: { className?: string }) {
  return (
    <svg className={className} viewBox="0 0 24 24" fill="currentColor">
      <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.23-.015-2.235-3.015.555-3.795-.735-4.035-1.41-.135-.345-.72-1.41-1.23-1.695-.42-.225-1.02-.78-.015-.795.945-.015 1.62.87 1.845 1.23 1.08 1.815 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.18 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405s2.04.135 3 .405c2.295-1.56 3.3-1.23 3.3-1.23.66 1.65.24 2.88.12 3.18.765.84 1.23 1.905 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.3 0 .315.225.69.825.57A12.02 12.02 0 0024 12c0-6.63-5.37-12-12-12z" />
    </svg>
  );
}

/**
 * Google Icon
 */
function GoogleIcon({ className }: { className?: string }) {
  return (
    <svg className={className} viewBox="0 0 24 24">
      <path
        fill="#4285F4"
        d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
      />
      <path
        fill="#34A853"
        d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
      />
      <path
        fill="#FBBC05"
        d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
      />
      <path
        fill="#EA4335"
        d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
      />
    </svg>
  );
}

export default function AccountSettingsPage() {
  return (
    <Layout title="Account Settings" description="Manage your account settings">
      <RequireAuth>
        <AccountSettingsContent />
      </RequireAuth>
    </Layout>
  );
}
