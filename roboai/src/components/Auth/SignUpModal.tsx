/**
 * SignUpModal Component
 *
 * Modal dialog for new user registration with email/password.
 * Implements T024-T030, T039 from the task list.
 *
 * Features:
 * - Email/password/name form fields
 * - Client-side validation (email format, password min 8 chars)
 * - Loading state during sign-up
 * - Success message with email verification instructions
 * - Error handling for common scenarios (email exists, weak password)
 * - Resend verification email option
 */
import React, { useState, useCallback } from "react";
import { signUp } from "@/lib/auth";
import authClient from "@/lib/auth";

interface SignUpModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignIn?: () => void;
}

interface FormState {
  name: string;
  email: string;
  password: string;
}

interface FormErrors {
  name?: string;
  email?: string;
  password?: string;
  general?: string;
}

export function SignUpModal({
  isOpen,
  onClose,
  onSwitchToSignIn,
}: SignUpModalProps) {
  const [formData, setFormData] = useState<FormState>({
    name: "",
    email: "",
    password: "",
  });
  const [errors, setErrors] = useState<FormErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);
  const [isResending, setIsResending] = useState(false);

  /**
   * Validate form fields
   * Returns true if valid, false otherwise
   */
  const validateForm = useCallback((): boolean => {
    const newErrors: FormErrors = {};

    // Name validation
    if (!formData.name.trim()) {
      newErrors.name = "Name is required";
    } else if (formData.name.trim().length < 2) {
      newErrors.name = "Name must be at least 2 characters";
    }

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!formData.email) {
      newErrors.email = "Email is required";
    } else if (!emailRegex.test(formData.email)) {
      newErrors.email = "Please enter a valid email address";
    }

    // Password validation (min 8 chars per research.md)
    if (!formData.password) {
      newErrors.password = "Password is required";
    } else if (formData.password.length < 8) {
      newErrors.password = "Password must be at least 8 characters";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  }, [formData]);

  /**
   * Handle form input changes
   */
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    // Clear field-specific error when user starts typing
    if (errors[name as keyof FormErrors]) {
      setErrors((prev) => ({ ...prev, [name]: undefined }));
    }
  };

  /**
   * Handle form submission
   * Calls signUp.email() with callbacks for request lifecycle
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setErrors({});

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      await signUp.email(
        {
          email: formData.email,
          password: formData.password,
          name: formData.name.trim(),
        },
        {
          onRequest: () => {
            console.log("[SignUp] Submitting registration...");
          },
          onSuccess: () => {
            console.log("[SignUp] Registration successful");
            setIsSuccess(true);
          },
          onError: (ctx) => {
            console.error("[SignUp] Registration failed:", ctx.error);
            handleSignUpError(ctx.error);
          },
        }
      );
    } catch (err) {
      console.error("[SignUp] Unexpected error:", err);
      setErrors({
        general: "An unexpected error occurred. Please try again.",
      });
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Map Better-Auth errors to user-friendly messages
   */
  const handleSignUpError = (error: { message?: string; status?: number }) => {
    const message = error.message?.toLowerCase() || "";

    if (
      message.includes("email already exists") ||
      message.includes("user already exists")
    ) {
      setErrors({
        email: "An account with this email already exists",
      });
    } else if (message.includes("password")) {
      setErrors({
        password: error.message || "Password does not meet requirements",
      });
    } else if (error.status === 429) {
      setErrors({
        general: "Too many attempts. Please try again later.",
      });
    } else {
      setErrors({
        general: error.message || "Registration failed. Please try again.",
      });
    }
  };

  /**
   * Handle resend verification email
   */
  const handleResendVerification = async () => {
    setIsResending(true);
    try {
      // Access sendVerificationEmail from authClient if available
      // This method requires backend email configuration
      if ("sendVerificationEmail" in authClient) {
        await (authClient as any).sendVerificationEmail({
          email: formData.email,
          callbackURL: "/dashboard",
        });
        alert("Verification email sent! Check your inbox.");
      } else {
        alert("Email verification is not configured. Please contact support.");
      }
    } catch (err) {
      console.error("[SignUp] Failed to resend verification:", err);
      alert("Failed to send verification email. Please try again.");
    } finally {
      setIsResending(false);
    }
  };

  /**
   * Reset form state when closing
   */
  const handleClose = () => {
    setFormData({ name: "", email: "", password: "" });
    setErrors({});
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
        aria-labelledby="signup-title"
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
          /* Success state - email verification instructions */
          <div className="auth-modal__success">
            <div className="auth-modal__success-icon">✓</div>
            <h2 id="signup-title" className="auth-modal__title">
              Check your email
            </h2>
            <p className="auth-modal__text">
              We sent a verification link to{" "}
              <strong>{formData.email}</strong>
            </p>
            <p className="auth-modal__text">
              Click the link in the email to verify your account and sign in.
            </p>
            <div className="auth-modal__actions">
              <button
                type="button"
                className="auth-btn auth-btn--secondary"
                onClick={handleResendVerification}
                disabled={isResending}
              >
                {isResending ? "Sending..." : "Resend verification email"}
              </button>
              <button
                type="button"
                className="auth-btn auth-btn--primary"
                onClick={handleClose}
              >
                Done
              </button>
            </div>
          </div>
        ) : (
          /* Sign-up form */
          <>
            <h2 id="signup-title" className="auth-modal__title">
              Create your account
            </h2>

            {/* SSO buttons will be added here in Phase 5 (US3) */}
            {/* <SSOButtons /> */}
            {/* <div className="auth-modal__divider">Or continue with email</div> */}

            <form onSubmit={handleSubmit} className="auth-form" noValidate>
              {/* General error message */}
              {errors.general && (
                <div className="auth-form__error auth-form__error--general" role="alert">
                  {errors.general}
                </div>
              )}

              {/* Name field */}
              <div className="auth-form__field">
                <label htmlFor="signup-name" className="auth-form__label">
                  Name
                </label>
                <input
                  id="signup-name"
                  type="text"
                  name="name"
                  value={formData.name}
                  onChange={handleChange}
                  className={`auth-form__input ${errors.name ? "auth-form__input--error" : ""}`}
                  placeholder="Your name"
                  disabled={isLoading}
                  required
                  autoComplete="name"
                />
                {errors.name && (
                  <span className="auth-form__error" role="alert">
                    {errors.name}
                  </span>
                )}
              </div>

              {/* Email field */}
              <div className="auth-form__field">
                <label htmlFor="signup-email" className="auth-form__label">
                  Email
                </label>
                <input
                  id="signup-email"
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
                <label htmlFor="signup-password" className="auth-form__label">
                  Password
                </label>
                <input
                  id="signup-password"
                  type="password"
                  name="password"
                  value={formData.password}
                  onChange={handleChange}
                  className={`auth-form__input ${errors.password ? "auth-form__input--error" : ""}`}
                  placeholder="At least 8 characters"
                  disabled={isLoading}
                  required
                  minLength={8}
                  autoComplete="new-password"
                />
                {errors.password && (
                  <span className="auth-form__error" role="alert">
                    {errors.password}
                  </span>
                )}
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
                    Creating account...
                  </span>
                ) : (
                  "Sign Up"
                )}
              </button>
            </form>

            {/* Link to sign-in */}
            <p className="auth-modal__footer">
              Already have an account?{" "}
              <button
                type="button"
                className="auth-link"
                onClick={onSwitchToSignIn}
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

export default SignUpModal;
