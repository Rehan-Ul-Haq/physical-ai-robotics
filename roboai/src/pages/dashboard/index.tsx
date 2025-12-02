/**
 * Dashboard Page
 *
 * Protected page for authenticated users.
 * Implements T037-T038 from the task list.
 *
 * Features:
 * - Protected route (requires authentication)
 * - Displays user name and email
 * - Session info display
 * - Skeleton loader during session restoration
 */
import React from "react";
import Layout from "@theme/Layout";
import { RequireAuth } from "@/components/Auth";
import { useSession } from "@/lib/auth";

/**
 * Dashboard content component
 * Rendered only when user is authenticated
 */
function DashboardContent() {
  const { data: session, refetch } = useSession();

  if (!session) {
    return null;
  }

  const { user } = session;

  return (
    <div className="dashboard">
      <div className="dashboard__header">
        <h1 className="dashboard__title">Welcome, {user.name || "User"}!</h1>
        <p className="dashboard__subtitle">
          Here's your account overview
        </p>
      </div>

      <div className="dashboard__grid">
        {/* Profile Card */}
        <div className="dashboard__card">
          <h2 className="dashboard__card-title">Profile</h2>
          <div className="dashboard__profile">
            {user.image ? (
              <img
                src={user.image}
                alt={user.name || "User"}
                className="dashboard__avatar"
              />
            ) : (
              <div className="dashboard__avatar dashboard__avatar--placeholder">
                {(user.name || user.email || "U")[0].toUpperCase()}
              </div>
            )}
            <div className="dashboard__profile-info">
              <p className="dashboard__name">{user.name || "Not set"}</p>
              <p className="dashboard__email">{user.email}</p>
              <p className="dashboard__verified">
                {user.emailVerified ? (
                  <span className="dashboard__badge dashboard__badge--success">
                    ✓ Email verified
                  </span>
                ) : (
                  <span className="dashboard__badge dashboard__badge--warning">
                    ⚠ Email not verified
                  </span>
                )}
              </p>
            </div>
          </div>
          <a href="/profile" className="dashboard__link">
            Edit Profile →
          </a>
        </div>

        {/* Session Info Card */}
        <div className="dashboard__card">
          <h2 className="dashboard__card-title">Session</h2>
          <dl className="dashboard__info-list">
            <div className="dashboard__info-item">
              <dt>Session ID</dt>
              <dd className="dashboard__mono">
                {session.session.id.slice(0, 8)}...
              </dd>
            </div>
            <div className="dashboard__info-item">
              <dt>Expires</dt>
              <dd>
                {new Date(session.session.expiresAt).toLocaleDateString(
                  undefined,
                  {
                    year: "numeric",
                    month: "short",
                    day: "numeric",
                    hour: "2-digit",
                    minute: "2-digit",
                  }
                )}
              </dd>
            </div>
          </dl>
          <button
            type="button"
            className="dashboard__link"
            onClick={() => refetch()}
          >
            Refresh Session →
          </button>
        </div>

        {/* Quick Actions Card */}
        <div className="dashboard__card">
          <h2 className="dashboard__card-title">Quick Actions</h2>
          <div className="dashboard__actions">
            <a href="/settings/account" className="dashboard__action">
              <span className="dashboard__action-icon">⚙️</span>
              <span>Account Settings</span>
            </a>
            <a href="/docs" className="dashboard__action">
              <span className="dashboard__action-icon">📚</span>
              <span>Browse Lessons</span>
            </a>
            <a href="/" className="dashboard__action">
              <span className="dashboard__action-icon">🏠</span>
              <span>Go to Homepage</span>
            </a>
          </div>
        </div>
      </div>
    </div>
  );
}

/**
 * Loading skeleton for dashboard
 */
function DashboardSkeleton() {
  return (
    <div className="dashboard dashboard--loading">
      <div className="dashboard__header">
        <div className="skeleton skeleton--title" />
        <div className="skeleton skeleton--subtitle" />
      </div>
      <div className="dashboard__grid">
        <div className="dashboard__card">
          <div className="skeleton skeleton--card" />
        </div>
        <div className="dashboard__card">
          <div className="skeleton skeleton--card" />
        </div>
        <div className="dashboard__card">
          <div className="skeleton skeleton--card" />
        </div>
      </div>
    </div>
  );
}

/**
 * Dashboard Page Component
 */
export default function DashboardPage() {
  return (
    <Layout
      title="Dashboard"
      description="Your personal dashboard"
    >
      <main className="container margin-vert--lg">
        <RequireAuth fallback={<DashboardSkeleton />}>
          <DashboardContent />
        </RequireAuth>
      </main>
    </Layout>
  );
}
