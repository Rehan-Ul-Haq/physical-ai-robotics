/**
 * Better-Auth React Client Configuration
 *
 * This module creates and configures the Better-Auth client for the roboai frontend.
 * It provides React hooks and methods for authentication operations.
 *
 * @see https://www.better-auth.com/docs/integrations/react
 */
import { createAuthClient } from "better-auth/react";

/**
 * Auth service URL from environment variable
 * In development: http://localhost:8002
 * In production: https://api.yourdomain.com
 */
const authServiceURL =
  process.env.REACT_APP_AUTH_SERVICE_URL || "http://localhost:8002";

/**
 * Better-Auth client instance configured for the roboai application
 *
 * Features:
 * - React hooks (useSession, useListAccounts)
 * - Email/password authentication
 * - OAuth social providers (GitHub, Google)
 * - Session management with httpOnly cookies
 * - Global error handling for rate limiting and unauthorized access
 */
export const authClient = createAuthClient({
  baseURL: authServiceURL,

  fetchOptions: {
    // Required for cross-origin cookie handling
    credentials: "include",

    /**
     * Global error handler for auth requests
     * Handles common error scenarios:
     * - 401: Session expired or unauthorized
     * - 403: Email not verified
     * - 429: Rate limit exceeded
     */
    onError(error) {
      const status = error.error?.status;

      switch (status) {
        case 401:
          console.error("[Auth] Unauthorized - session may have expired");
          break;
        case 403:
          console.error("[Auth] Forbidden - email verification required");
          break;
        case 429:
          console.error("[Auth] Rate limit exceeded - please try again later");
          break;
        default:
          console.error("[Auth] Error:", error.error?.message || "Unknown error");
      }
    },

    /**
     * Optional success callback for analytics/logging
     */
    onSuccess(data) {
      if (process.env.NODE_ENV === "development") {
        console.log("[Auth] Action successful");
      }
    },
  },
});

/**
 * Exported auth methods and hooks for use throughout the application
 *
 * Hooks:
 * - useSession: Get current session state with loading/error states
 *
 * Sign-in methods:
 * - signIn.email: Email/password authentication
 * - signIn.social: OAuth with GitHub/Google
 *
 * Sign-up methods:
 * - signUp.email: Create account with email/password
 *
 * Account management:
 * - signOut: End current session
 *
 * Note: Password reset methods (forgetPassword, resetPassword) and email
 * verification (sendVerificationEmail) require backend configuration.
 * They are accessed via authClient directly when available.
 */
export const { signIn, signUp, signOut, useSession } = authClient;

// Re-export the full client for advanced use cases and methods
// that may be available depending on backend configuration
export default authClient;

// Type exports for consumers
export type { Session } from "better-auth/types";
