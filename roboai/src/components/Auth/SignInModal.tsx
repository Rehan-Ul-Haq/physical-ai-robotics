/**
 * SignInModal Component
 *
 * Modal dialog for existing user authentication with email/password.
 * Implements T040-T051 from the task list.
 *
 * Features:
 * - Email/password form fields
 * - "Remember me" checkbox for session persistence
 * - Loading state during sign-in
 * - Error handling without email enumeration
 * - Link to sign-up modal
 * - Link to password reset
 */
import React, { useState, useCallback, useEffect } from "react";
import { signIn, useSession, authServiceURL } from "@/lib/auth";
import { SSOButtons } from "./SSOButtons";

interface SignInModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignUp?: () => void;
  onForgotPassword?: () => void;
  redirectTo?: string;
}

interface FormState {
  email: string;
  password: string;
  rememberMe: boolean;
}

interface FormErrors {
  email?: string;
  password?: string;
  general?: string;
}

export function SignInModal({
  isOpen,
  onClose,
  onSwitchToSignUp,
  onForgotPassword,
  redirectTo,
}: SignInModalProps) {
  const [formData, setFormData] = useState<FormState>({
    email: "",
    password: "",
    rememberMe: false,
  });
  const [errors, setErrors] = useState<FormErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [isResending, setIsResending] = useState(false);
  const [showResendVerification, setShowResendVerification] = useState(false);

  // Check for active session and auto-redirect (T047)
  const { data: session, isPending } = useSession();

  useEffect(() => {
    if (!isPending && session && isOpen) {
      // User is already authenticated, close modal and stay on page
      if (redirectTo) {
        window.location.href = redirectTo;
      } else {
        // Just close modal and let the page re-render with auth state
        onClose();
      }
    }
  }, [session, isPending, isOpen, redirectTo, onClose]);

  /**
   * Validate form fields
   */
  const validateForm = useCallback((): boolean => {
    const newErrors: FormErrors = {};

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!formData.email) {
      newErrors.email = "Email is required";
    } else if (!emailRegex.test(formData.email)) {
      newErrors.email = "Please enter a valid email address";
    }

    // Password validation
    if (!formData.password) {
      newErrors.password = "Password is required";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  }, [formData]);

  /**
   * Handle form input changes
   */
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value, type, checked } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: type === "checkbox" ? checked : value,
    }));
    // Clear field-specific error when user starts typing
    if (errors[name as keyof FormErrors]) {
      setErrors((prev) => ({ ...prev, [name]: undefined }));
    }
    // Hide resend verification option when email changes
    if (name === "email") {
      setShowResendVerification(false);
    }
  };

  /**
   * Handle form submission
   * Calls signIn.email() with rememberMe option
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setErrors({});
    setShowResendVerification(false);

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      await signIn.email(
        {
          email: formData.email,
          password: formData.password,
          rememberMe: formData.rememberMe,
        },
        {
          onRequest: () => {
            console.log("[SignIn] Authenticating...");
          },
          onSuccess: () => {
            console.log("[SignIn] Authentication successful");
            // Close modal and stay on page, or redirect if specified
            if (redirectTo) {
              window.location.href = redirectTo;
            } else {
              // Reload to refresh auth state across the page
              window.location.reload();
            }
          },
          onError: (ctx) => {
            console.error("[SignIn] Authentication failed:", ctx.error);
            handleSignInError(ctx.error);
          },
        }
      );
    } catch (err) {
      console.error("[SignIn] Unexpected error:", err);
      setErrors({
        general: "An unexpected error occurred. Please try again.",
      });
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Map Better-Auth errors to user-friendly messages
   * Important: Don't reveal whether email exists (security best practice)
   */
  const handleSignInError = (error: { message?: string; status?: number }) => {
    const status = error.status;

    if (status === 403) {
      // Email not verified (T050)
      setErrors({
        general: "Please verify your email address before signing in.",
      });
      setShowResendVerification(true);
    } else if (status === 429) {
      setErrors({
        general: "Too many attempts. Please try again later.",
      });
    } else {
      // Generic error message to prevent email enumeration (T045)
      setErrors({
        general: "Invalid email or password",
      });
    }
  };

  /**
   * Handle resend verification email
   */
  const handleResendVerification = async () => {
    if (!formData.email) {
      alert("Please enter your email address first.");
      return;
    }
    
    setIsResending(true);
    try {
      // Use direct fetch to Better-Auth send-verification-email endpoint
      const response = await fetch(`${authServiceURL}/api/auth/send-verification-email`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include",
        body: JSON.stringify({
          email: formData.email,
          callbackURL: `${window.location.origin}/physical-ai-robotics/`,
        }),
      });
      
      if (response.ok) {
        alert("Verification email sent! Check your inbox.");
      } else {
        const data = await response.json().catch(() => ({}));
        console.error("[SignIn] Resend verification error:", data);
        alert(data.message || "Failed to send verification email.");
      }
    } catch (err) {
      console.error("[SignIn] Failed to resend verification:", err);
      alert("Failed to send verification email. Please try again.");
    } finally {
      setIsResending(false);
    }
  };

  /**
   * Reset form state when closing
   */
  const handleClose = () => {
    setFormData({ email: "", password: "", rememberMe: false });
    setErrors({});
    setIsLoading(false);
    setIsResending(false);
    setShowResendVerification(false);
    onClose();
  };

  if (!isOpen) return null;

  // Show loading if checking session
  if (isPending) {
    return (
      <div className="auth-modal-overlay">
        <div className="auth-modal">
          <div className="auth-modal__loading">Checking session...</div>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-modal-overlay" onClick={handleClose}>
      <div
        className="auth-modal"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="signin-title"
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

        <h2 id="signin-title" className="auth-modal__title">
          Welcome back
        </h2>

        {/* SSO buttons (T066) */}
        <SSOButtons />
        <div className="auth-modal__divider">Or continue with email</div>

        <form onSubmit={handleSubmit} className="auth-form" noValidate>
          {/* General error message */}
          {errors.general && (
            <div className="auth-form__error auth-form__error--general" role="alert">
              {errors.general}
              {showResendVerification && (
                <button
                  type="button"
                  className="auth-link auth-link--inline"
                  onClick={handleResendVerification}
                  disabled={isResending}
                >
                  {isResending ? "Sending..." : "Resend verification email"}
                </button>
              )}
            </div>
          )}

          {/* Email field */}
          <div className="auth-form__field">
            <label htmlFor="signin-email" className="auth-form__label">
              Email
            </label>
            <input
              id="signin-email"
              type="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              className={`auth-form__input ${errors.email ? "auth-form__input--error" : ""}`}
              placeholder="you@example.com"
              disabled={isLoading}
              required
              autoComplete="email"
            />
            {errors.email && (
              <span className="auth-form__error" role="alert">
                {errors.email}
              </span>
            )}
          </div>

          {/* Password field */}
          <div className="auth-form__field">
            <label htmlFor="signin-password" className="auth-form__label">
              Password
            </label>
            <input
              id="signin-password"
              type="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              className={`auth-form__input ${errors.password ? "auth-form__input--error" : ""}`}
              placeholder="Enter your password"
              disabled={isLoading}
              required
              autoComplete="current-password"
            />
            {errors.password && (
              <span className="auth-form__error" role="alert">
                {errors.password}
              </span>
            )}
          </div>

          {/* Remember me and forgot password row */}
          <div className="auth-form__row">
            <label className="auth-form__checkbox">
              <input
                type="checkbox"
                name="rememberMe"
                checked={formData.rememberMe}
                onChange={handleChange}
                disabled={isLoading}
              />
              <span>Remember me</span>
            </label>
            <button
              type="button"
              className="auth-link"
              onClick={onForgotPassword}
            >
              Forgot password?
            </button>
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
                Signing in...
              </span>
            ) : (
              "Sign In"
            )}
          </button>
        </form>

        {/* Link to sign-up (T051) */}
        <p className="auth-modal__footer">
          Don't have an account?{" "}
          <button
            type="button"
            className="auth-link"
            onClick={onSwitchToSignUp}
          >
            Sign up
          </button>
        </p>
      </div>
    </div>
  );
}

export default SignInModal;
