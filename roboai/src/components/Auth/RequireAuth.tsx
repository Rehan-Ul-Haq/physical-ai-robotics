/**
 * RequireAuth Component
 *
 * Higher-Order Component for protecting routes and content.
 * Implements T069-T072, T080-T081 from the task list.
 *
 * Features:
 * - Session check with loading state
 * - Redirect to sign-in for unauthenticated users
 * - Preserves intended destination URL
 * - Customizable fallback during loading
 * - Handles session expiration gracefully
 */
import React, { useEffect, ReactNode } from "react";
import { useSession } from "@/lib/auth";
import { useHistory, useLocation } from "@docusaurus/router";

interface RequireAuthProps {
  /** Content to render when authenticated */
  children: ReactNode;
  /** URL to redirect unauthenticated users (default: /sign-in) */
  redirectTo?: string;
  /** Content to show while checking session */
  fallback?: ReactNode;
  /** Callback when auth is required but user is not authenticated */
  onAuthRequired?: () => void;
}

/**
 * Default loading fallback component
 */
function DefaultFallback() {
  return (
    <div className="require-auth__loading">
      <div className="require-auth__spinner" aria-hidden="true" />
      <p>Loading...</p>
    </div>
  );
}

/**
 * Session expired modal component
 */
function SessionExpiredModal({ onReauth }: { onReauth: () => void }) {
  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal" role="alertdialog" aria-labelledby="session-expired-title">
        <h2 id="session-expired-title" className="auth-modal__title">
          Session Expired
        </h2>
        <p className="auth-modal__text">
          Your session has expired. Please sign in again to continue.
        </p>
        <button
          type="button"
          className="auth-btn auth-btn--primary auth-btn--full"
          onClick={onReauth}
        >
          Sign In Again
        </button>
      </div>
    </div>
  );
}

export function RequireAuth({
  children,
  redirectTo = "/sign-in",
  fallback = <DefaultFallback />,
  onAuthRequired,
}: RequireAuthProps) {
  const { data: session, isPending, error } = useSession();
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    // Wait for session check to complete
    if (isPending) return;

    // If no session and no error, user is not authenticated
    if (!session && !error) {
      // Store intended destination for redirect after auth (T072)
      const currentPath = location.pathname + location.search;
      if (typeof window !== "undefined") {
        sessionStorage.setItem("auth_redirect_url", currentPath);
      }

      // Call optional callback
      if (onAuthRequired) {
        onAuthRequired();
      } else {
        // Default: redirect to sign-in
        history.push(redirectTo);
      }
    }
  }, [isPending, session, error, history, redirectTo, location, onAuthRequired]);

  // Show loading state while checking session (prevents FOUC - T117)
  if (isPending) {
    return <>{fallback}</>;
  }

  // Handle session error (e.g., session expired while using protected feature - T080)
  if (error) {
    const handleReauth = () => {
      // Store current location for redirect after re-auth
      const currentPath = location.pathname + location.search;
      if (typeof window !== "undefined") {
        sessionStorage.setItem("auth_redirect_url", currentPath);
      }
      history.push(redirectTo);
    };

    return <SessionExpiredModal onReauth={handleReauth} />;
  }

  // User is not authenticated, will redirect via useEffect
  if (!session) {
    return <>{fallback}</>;
  }

  // User is authenticated, render protected content
  return <>{children}</>;
}

/**
 * Custom hook for requiring authentication
 * Alternative to RequireAuth component for inline usage
 *
 * @example
 * function ProfilePage() {
 *   const { session, isPending } = useRequireAuth();
 *   if (isPending) return <Loading />;
 *   if (!session) return null;
 *   return <div>Hello, {session.user.name}</div>;
 * }
 */
export function useRequireAuth(redirectTo: string = "/sign-in") {
  const { data: session, isPending, error, refetch } = useSession();
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    if (!isPending && !session && !error) {
      // Store intended destination
      const currentPath = location.pathname + location.search;
      if (typeof window !== "undefined") {
        sessionStorage.setItem("auth_redirect_url", currentPath);
      }
      history.push(redirectTo);
    }
  }, [isPending, session, error, history, redirectTo, location]);

  return { session, isPending, error, refetch };
}

/**
 * Utility to get stored redirect URL after authentication
 */
export function getAuthRedirectUrl(defaultUrl: string = "/dashboard"): string {
  if (typeof window === "undefined") return defaultUrl;

  const storedUrl = sessionStorage.getItem("auth_redirect_url");
  if (storedUrl) {
    sessionStorage.removeItem("auth_redirect_url");
    return storedUrl;
  }
  return defaultUrl;
}

export default RequireAuth;
