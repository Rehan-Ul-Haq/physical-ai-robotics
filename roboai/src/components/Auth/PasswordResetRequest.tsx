/**
 * PasswordResetRequest Component
 *
 * Form for requesting a password reset email.
 * Implements T099-T102 from the task list.
 *
 * Features:
 * - Email input form
 * - Submit handler calling forgetPassword()
 * - Success message without email enumeration
 * - Loading state during request
 */
import React, { useState } from "react";
import authClient from "@/lib/auth";

interface PasswordResetRequestProps {
  isOpen: boolean;
  onClose: () => void;
  onBackToSignIn?: () => void;
}

export function PasswordResetRequest({
  isOpen,
  onClose,
  onBackToSignIn,
}: PasswordResetRequestProps) {
  const [email, setEmail] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);
  const [error, setError] = useState("");

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    // Basic email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!email || !emailRegex.test(email)) {
      setError("Please enter a valid email address");
      return;
    }

    setIsLoading(true);

    try {
      // Access forgetPassword from authClient if available
      // This method requires backend email configuration
      if ("forgetPassword" in authClient) {
        await (authClient as any).forgetPassword({
          email: email,
          redirectTo: "/reset-password",
        });
      }
      // Always show success to prevent email enumeration (T102)
      setIsSuccess(true);
    } catch (err) {
      console.error("[PasswordReset] Request failed:", err);
      // Still show success message to prevent enumeration
      setIsSuccess(true);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Reset form state when closing
   */
  const handleClose = () => {
    setEmail("");
    setError("");
    setIsLoading(false);
    setIsSuccess(false);
    onClose();
  };

  if (!isOpen) return null;

  return (
    <div className="auth-modal-overlay" onClick={handleClose}>
      <div
        className="auth-modal"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="password-reset-title"
      >
        {/* Close button */}
        <button
          className="auth-modal__close"
          onClick={handleClose}
          aria-label="Close"
          type="button"
        >
          ×
        </button>

        {isSuccess ? (
          /* Success state - no email enumeration */
          <div className="auth-modal__success">
            <div className="auth-modal__success-icon">✉️</div>
            <h2 id="password-reset-title" className="auth-modal__title">
              Check your email
            </h2>
            <p className="auth-modal__text">
              If an account exists for <strong>{email}</strong>, you'll receive
              a password reset link shortly.
            </p>
            <p className="auth-modal__text auth-modal__text--muted">
              Didn't receive an email? Check your spam folder or try again with
              a different email address.
            </p>
            <div className="auth-modal__actions">
              <button
                type="button"
                className="auth-btn auth-btn--primary auth-btn--full"
                onClick={onBackToSignIn}
              >
                Back to Sign In
              </button>
            </div>
          </div>
        ) : (
          /* Request form */
          <>
            <h2 id="password-reset-title" className="auth-modal__title">
              Reset your password
            </h2>
            <p className="auth-modal__text">
              Enter your email address and we'll send you a link to reset your
              password.
            </p>

            <form onSubmit={handleSubmit} className="auth-form" noValidate>
              {/* Error message */}
              {error && (
                <div className="auth-form__error auth-form__error--general" role="alert">
                  {error}
                </div>
              )}

              {/* Email field */}
              <div className="auth-form__field">
                <label htmlFor="reset-email" className="auth-form__label">
                  Email
                </label>
                <input
                  id="reset-email"
                  type="email"
                  name="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  className={`auth-form__input ${error ? "auth-form__input--error" : ""}`}
                  placeholder="you@example.com"
                  disabled={isLoading}
                  required
                  autoComplete="email"
                  autoFocus
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
                    Sending...
                  </span>
                ) : (
                  "Send Reset Link"
                )}
              </button>
            </form>

            {/* Link back to sign-in */}
            <p className="auth-modal__footer">
              Remember your password?{" "}
              <button
                type="button"
                className="auth-link"
                onClick={onBackToSignIn}
              >
                Sign in
              </button>
            </p>
          </>
        )}
      </div>
    </div>
  );
}

export default PasswordResetRequest;
