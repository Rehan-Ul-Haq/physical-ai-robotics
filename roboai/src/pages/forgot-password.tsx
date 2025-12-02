/**
 * ForgotPassword Page
 *
 * Page for requesting a password reset link.
 * Redirects here when reset link is expired.
 */
import React, { useState } from "react";
import Layout from "@theme/Layout";
import authClient from "@/lib/auth";

export default function ForgotPasswordPage() {
  const [email, setEmail] = useState("");
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);

  /**
   * Validate email format
   */
  const validateEmail = (): boolean => {
    if (!email) {
      setError("Email is required");
      return false;
    }
    if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email)) {
      setError("Please enter a valid email address");
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

    if (!validateEmail()) {
      return;
    }

    setIsLoading(true);

    try {
      // Access forgetPassword from authClient if available
      if ("forgetPassword" in authClient) {
        await (authClient as any).forgetPassword({
          email: email,
          redirectTo: `${window.location.origin}/reset-password`,
        });
      }
      // Always show success to prevent email enumeration
      setIsSuccess(true);
    } catch (err: any) {
      console.error("[ForgotPassword] Request failed:", err);
      // Show success message anyway to prevent email enumeration attacks
      setIsSuccess(true);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Go back to home/sign-in
   */
  const handleGoBack = () => {
    window.location.href = "/";
  };

  return (
    <Layout
      title="Forgot Password"
      description="Request a password reset link"
    >
      <main className="container margin-vert--xl">
        <div className="reset-password-page">
          {isSuccess ? (
            /* Success state */
            <div className="reset-password-card reset-password-card--success">
              <div className="reset-password-icon">📧</div>
              <h1 className="reset-password-title">Check Your Email</h1>
              <p className="reset-password-text">
                If an account with that email exists, we've sent a password reset link.
              </p>
              <p className="reset-password-text reset-password-text--muted">
                The link will expire in 1 hour. Be sure to check your spam folder.
              </p>
              <button
                type="button"
                className="auth-btn auth-btn--secondary auth-btn--full"
                onClick={handleGoBack}
              >
                Back to Home
              </button>
            </div>
          ) : (
            /* Request form */
            <div className="reset-password-card">
              <h1 className="reset-password-title">Forgot Password</h1>
              <p className="reset-password-text">
                Enter your email address and we'll send you a link to reset your password.
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
                  <label htmlFor="email" className="auth-form__label">
                    Email Address
                  </label>
                  <input
                    id="email"
                    type="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    className="auth-form__input"
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

                {/* Back link */}
                <button
                  type="button"
                  className="auth-btn auth-btn--secondary auth-btn--full"
                  onClick={handleGoBack}
                >
                  Back to Sign In
                </button>
              </form>
            </div>
          )}
        </div>
      </main>
    </Layout>
  );
}
