/**
 * Profile Page
 *
 * User profile page for viewing and updating profile information.
 * Implements T085-T087 from the task list.
 *
 * Features:
 * - Display user profile information
 * - Update display name
 * - Update profile picture (future)
 */
import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useSession } from "@/lib/auth";
import authClient from "@/lib/auth";
import { RequireAuth } from "@/components/Auth";
import "./profile.css";

function ProfileContent() {
  const { data: session, refetch } = useSession();
  const [displayName, setDisplayName] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");

  // Initialize form with current user data
  useEffect(() => {
    if (session?.user) {
      setDisplayName(session.user.name || "");
    }
  }, [session]);

  /**
   * Handle profile update
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setSuccess("");

    if (!displayName.trim()) {
      setError("Display name is required");
      return;
    }

    setIsLoading(true);

    try {
      // Access updateUser from authClient if available
      if ("updateUser" in authClient) {
        await (authClient as any).updateUser({
          name: displayName.trim(),
        });
        setSuccess("Profile updated successfully!");
        // Refetch session to update UI
        await refetch();
      } else {
        setError("Profile update is not configured.");
      }
    } catch (err: any) {
      console.error("[Profile] Update failed:", err);
      setError(err?.message || "Failed to update profile. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  if (!session) {
    return null;
  }

  const { user } = session;
  const initials = (user.name || user.email?.split("@")[0] || "U")
    .split(" ")
    .map((n) => n[0])
    .join("")
    .toUpperCase()
    .slice(0, 2);

  return (
    <div className="profile-page">
      <div className="profile-card">
        <h1 className="profile-title">Profile</h1>

        {/* Avatar Section */}
        <div className="profile-avatar-section">
          {user.image ? (
            <img
              src={user.image}
              alt={user.name || "Profile"}
              className="profile-avatar"
            />
          ) : (
            <div className="profile-avatar profile-avatar--initials">
              {initials}
            </div>
          )}
          {/* Future: Add avatar upload button */}
        </div>

        {/* Profile Form */}
        <form onSubmit={handleSubmit} className="profile-form">
          {/* Success message */}
          {success && (
            <div className="profile-message profile-message--success" role="status">
              {success}
            </div>
          )}

          {/* Error message */}
          {error && (
            <div className="profile-message profile-message--error" role="alert">
              {error}
            </div>
          )}

          {/* Email (read-only) */}
          <div className="profile-field">
            <label htmlFor="email" className="profile-label">
              Email
            </label>
            <input
              id="email"
              type="email"
              value={user.email || ""}
              className="profile-input profile-input--readonly"
              disabled
              readOnly
            />
            <span className="profile-hint">
              Email cannot be changed
              {user.emailVerified && (
                <span className="profile-badge profile-badge--verified">
                  ✓ Verified
                </span>
              )}
            </span>
          </div>

          {/* Display Name */}
          <div className="profile-field">
            <label htmlFor="displayName" className="profile-label">
              Display Name
            </label>
            <input
              id="displayName"
              type="text"
              value={displayName}
              onChange={(e) => setDisplayName(e.target.value)}
              className="profile-input"
              placeholder="Enter your display name"
              disabled={isLoading}
            />
          </div>

          {/* Account Created */}
          <div className="profile-field">
            <label className="profile-label">Member Since</label>
            <p className="profile-text">
              {user.createdAt
                ? new Date(user.createdAt).toLocaleDateString("en-US", {
                    year: "numeric",
                    month: "long",
                    day: "numeric",
                  })
                : "Unknown"}
            </p>
          </div>

          {/* Submit Button */}
          <button
            type="submit"
            className="profile-btn profile-btn--primary"
            disabled={isLoading}
          >
            {isLoading ? "Saving..." : "Save Changes"}
          </button>
        </form>

        {/* Links to other settings */}
        <div className="profile-links">
          <a href="/settings/account" className="profile-link">
            Account Settings →
          </a>
        </div>
      </div>
    </div>
  );
}

export default function ProfilePage() {
  return (
    <Layout title="Profile" description="Manage your profile">
      <RequireAuth>
        <ProfileContent />
      </RequireAuth>
    </Layout>
  );
}
