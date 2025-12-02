/**
 * AuthPromptModal Component
 *
 * Modal prompting users to sign in when accessing protected content.
 * Implements T073-T075 from the task list.
 *
 * Features:
 * - Customizable message for different protected features
 * - Sign in and sign up options
 * - Preserves intended destination for redirect after auth
 */
import React from "react";

interface AuthPromptModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSignIn: () => void;
  onSignUp: () => void;
  /** Title of the modal */
  title?: string;
  /** Message explaining why auth is required */
  message?: string;
  /** Feature name for analytics/tracking */
  feature?: string;
}

export function AuthPromptModal({
  isOpen,
  onClose,
  onSignIn,
  onSignUp,
  title = "Sign in required",
  message = "Please sign in to access this feature.",
  feature,
}: AuthPromptModalProps) {
  if (!isOpen) return null;

  // Log feature access attempt for analytics
  if (feature) {
    console.log(`[AuthPrompt] User attempted to access: ${feature}`);
  }

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div
        className="auth-modal"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="auth-prompt-title"
      >
        {/* Close button */}
        <button
          className="auth-modal__close"
          onClick={onClose}
          aria-label="Close"
          type="button"
        >
          ×
        </button>

        {/* Lock icon */}
        <div className="auth-modal__icon">
          <LockIcon />
        </div>

        <h2 id="auth-prompt-title" className="auth-modal__title">
          {title}
        </h2>

        <p className="auth-modal__text">{message}</p>

        <div className="auth-modal__actions">
          <button
            type="button"
            className="auth-btn auth-btn--primary auth-btn--full"
            onClick={onSignIn}
          >
            Sign In
          </button>
          <button
            type="button"
            className="auth-btn auth-btn--secondary auth-btn--full"
            onClick={onSignUp}
          >
            Create Account
          </button>
        </div>

        <p className="auth-modal__footer auth-modal__footer--centered">
          <button type="button" className="auth-link" onClick={onClose}>
            Maybe later
          </button>
        </p>
      </div>
    </div>
  );
}

/**
 * Lock icon for auth prompt
 */
function LockIcon() {
  return (
    <svg
      width="48"
      height="48"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
      <path d="M7 11V7a5 5 0 0 1 10 0v4" />
    </svg>
  );
}

export default AuthPromptModal;
