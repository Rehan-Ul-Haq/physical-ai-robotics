/**
 * PasswordResetConfirm Page
 *
 * Page for setting a new password using a reset token.
 * Implements T105-T112 from the task list.
 *
 * Features:
 * - New password input form
 * - Token validation from URL
 * - resetPassword() call
 * - Expired/used link handling
 * - Redirect to sign-in after success
 */
import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useLocation } from "@docusaurus/router";
import authClient from "@/lib/auth";

export default function ResetPasswordPage() {
  const location = useLocation();
  const [token, setToken] = useState<string | null>(null);
  const [newPassword, setNewPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);
  const [isInvalidToken, setIsInvalidToken] = useState(false);

  // Extract token from URL query params
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const tokenParam = params.get("token");
    if (tokenParam) {
      setToken(tokenParam);
    } else {
      setIsInvalidToken(true);
    }
  }, [location.search]);

  /**
   * Validate password requirements
   */
  const validatePassword = (): boolean => {
    if (!newPassword) {
      setError("Password is required");
      return false;
    }
    if (newPassword.length < 8) {
      setError("Password must be at least 8 characters");
      return false;
    }
    if (newPassword !== confirmPassword) {
      setError("Passwords do not match");
      return false;
    }
    return true;
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (!validatePassword()) {
      return;
    }

    if (!token) {
      setError("Invalid reset link. Please request a new one.");
      return;
    }

    setIsLoading(true);

    try {
      // Access resetPassword from authClient if available
      if ("resetPassword" in authClient) {
        await (authClient as any).resetPassword({
          newPassword: newPassword,
          token: token,
        });
        setIsSuccess(true);
      } else {
        setError("Password reset is not configured. Please contact support.");
      }
    } catch (err: any) {
      console.error("[PasswordReset] Reset failed:", err);
      
      // Handle specific error cases (T109, T110)
      const message = err?.message?.toLowerCase() || "";
      if (message.includes("expired")) {
        setError("This reset link has expired. Please request a new one.");
        setIsInvalidToken(true);
      } else if (message.includes("used") || message.includes("invalid")) {
        setError("This reset link has already been used or is invalid.");
        setIsInvalidToken(true);
      } else {
        setError("Failed to reset password. Please try again.");
      }
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Redirect to sign-in page
   */
  const handleGoToSignIn = () => {
    window.location.href = "/";
  };

  /**
   * Request new reset link
   */
  const handleRequestNewLink = () => {
    window.location.href = "/forgot-password";
  };

  return (
    <Layout
      title="Reset Password"
      description="Set a new password for your account"
    >
      <main className="container margin-vert--xl">
        <div className="reset-password-page">
          {isSuccess ? (
            /* Success state */
            <div className="reset-password-card reset-password-card--success">
              <div className="reset-password-icon">✓</div>
              <h1 className="reset-password-title">Password Reset Complete</h1>
              <p className="reset-password-text">
                Your password has been successfully reset.
                You can now sign in with your new password.
              </p>
              <button
                type="button"
                className="auth-btn auth-btn--primary auth-btn--full"
                onClick={handleGoToSignIn}
              >
                Go to Sign In
              </button>
            </div>
          ) : isInvalidToken ? (
            /* Invalid/expired token state */
            <div className="reset-password-card reset-password-card--error">
              <div className="reset-password-icon">⚠️</div>
              <h1 className="reset-password-title">Link Expired or Invalid</h1>
              <p className="reset-password-text">
                {error || "This password reset link has expired or is no longer valid."}
              </p>
              <p className="reset-password-text reset-password-text--muted">
                Password reset links expire after 1 hour for security reasons.
              </p>
              <div className="reset-password-actions">
                <button
                  type="button"
                  className="auth-btn auth-btn--primary auth-btn--full"
                  onClick={handleRequestNewLink}
                >
                  Request New Reset Link
                </button>
                <button
                  type="button"
                  className="auth-btn auth-btn--secondary auth-btn--full"
                  onClick={handleGoToSignIn}
                >
                  Back to Sign In
                </button>
              </div>
            </div>
          ) : (
            /* Reset form */
            <div className="reset-password-card">
              <h1 className="reset-password-title">Set New Password</h1>
              <p className="reset-password-text">
                Enter your new password below.
              </p>

              <form onSubmit={handleSubmit} className="auth-form" noValidate>
                {/* Error message */}
                {error && (
                  <div className="auth-form__error auth-form__error--general" role="alert">
                    {error}
                  </div>
                )}

                {/* New password field */}
                <div className="auth-form__field">
                  <label htmlFor="new-password" className="auth-form__label">
                    New Password
                  </label>
                  <input
                    id="new-password"
                    type="password"
                    value={newPassword}
                    onChange={(e) => setNewPassword(e.target.value)}
                    className="auth-form__input"
                    placeholder="At least 8 characters"
                    disabled={isLoading}
                    required
                    minLength={8}
                    autoComplete="new-password"
                    autoFocus
                  />
                </div>

                {/* Confirm password field */}
                <div className="auth-form__field">
                  <label htmlFor="confirm-password" className="auth-form__label">
                    Confirm Password
                  </label>
                  <input
                    id="confirm-password"
                    type="password"
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    className="auth-form__input"
                    placeholder="Confirm your password"
                    disabled={isLoading}
                    required
                    minLength={8}
                    autoComplete="new-password"
                  />
                </div>

                {/* Submit button */}
                <button
                  type="submit"
                  className="auth-btn auth-btn--primary auth-btn--full"
                  disabled={isLoading}
                >
                  {isLoading ? (
                    <span className="auth-btn__loading">
                      <span className="auth-btn__spinner" aria-hidden="true" />
                      Resetting...
                    </span>
                  ) : (
                    "Reset Password"
                  )}
                </button>
              </form>
            </div>
          )}
        </div>
      </main>
    </Layout>
  );
}
