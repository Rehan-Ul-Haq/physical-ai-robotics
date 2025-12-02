/**
 * AuthGuard Component
 *
 * Protects content from unauthenticated users.
 * Shows a compelling sign-up prompt instead of the protected content.
 *
 * Used to:
 * - Restrict lesson content to signed-in users
 * - Restrict ChatWidget to signed-in users
 */
import React, { useState } from "react";
import { useSession } from "@/lib/auth";
import { SignInModal } from "./SignInModal";
import { SignUpModal } from "./SignUpModal";
import { PasswordResetRequest } from "./PasswordResetRequest";
import "./Auth.css";

interface AuthGuardProps {
  /** Content to show when authenticated */
  children: React.ReactNode;
  /** Content type for messaging (e.g., "lesson", "chat assistant") */
  contentType?: string;
  /** Custom message to show unauthenticated users */
  message?: string;
  /** Whether to show a compact version */
  compact?: boolean;
}

type ModalState = "none" | "signIn" | "signUp" | "forgotPassword";

/**
 * Lock icon SVG
 */
const LockIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="1.5"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
    <path d="M7 11V7a5 5 0 0 1 10 0v4" />
  </svg>
);

/**
 * Book icon for lesson content
 */
const BookIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
    <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
  </svg>
);

/**
 * Robot icon for AI features
 */
const RobotIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <rect x="3" y="11" width="18" height="10" rx="2" />
    <circle cx="12" cy="5" r="2" />
    <path d="M12 7v4" />
    <line x1="8" y1="16" x2="8" y2="16" />
    <line x1="16" y1="16" x2="16" y2="16" />
  </svg>
);

/**
 * Sparkle icon for summaries
 */
const SparkleIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <path d="M12 3l1.5 4.5L18 9l-4.5 1.5L12 15l-1.5-4.5L6 9l4.5-1.5L12 3z" />
    <path d="M5 19l1 3 1-3 3-1-3-1-1-3-1 3-3 1 3 1z" />
  </svg>
);

export function AuthGuard({
  children,
  contentType = "content",
  message,
  compact = false,
}: AuthGuardProps): React.ReactElement {
  const { data: session, isPending } = useSession();
  const [activeModal, setActiveModal] = useState<ModalState>("none");

  const openSignIn = () => setActiveModal("signIn");
  const openSignUp = () => setActiveModal("signUp");
  const openForgotPassword = () => setActiveModal("forgotPassword");
  const closeModal = () => setActiveModal("none");

  // Show loading state while checking session
  if (isPending) {
    return (
      <div className={`auth-guard auth-guard--loading ${compact ? "auth-guard--compact" : ""}`}>
        <div className="auth-guard__spinner" />
        <span>Loading...</span>
      </div>
    );
  }

  // If authenticated, show the protected content
  if (session) {
    return <>{children}</>;
  }

  // Show compelling sign-up prompt for unauthenticated users
  return (
    <div className={`auth-guard ${compact ? "auth-guard--compact" : ""}`}>
      <div className="auth-guard__content">
        <div className="auth-guard__icon">
          <LockIcon />
        </div>
        <h3 className="auth-guard__title">
          {message || `This ${contentType} is for members only`}
        </h3>
        <p className="auth-guard__description">
          Join thousands of learners mastering Physical AI & Robotics. Create your free account in seconds.
        </p>
        
        {/* Feature highlights */}
        <div className="auth-guard__features">
          <div className="auth-guard__feature">
            <span className="auth-guard__feature-icon"><BookIcon /></span>
            <span>Full access to all lessons</span>
          </div>
          <div className="auth-guard__feature">
            <span className="auth-guard__feature-icon"><SparkleIcon /></span>
            <span>AI-generated summaries</span>
          </div>
          <div className="auth-guard__feature">
            <span className="auth-guard__feature-icon"><RobotIcon /></span>
            <span>Robo AI chat assistant</span>
          </div>
        </div>

        <div className="auth-guard__actions">
          <button
            type="button"
            className="auth-guard__btn auth-guard__btn--primary"
            onClick={openSignUp}
          >
            Create Free Account
          </button>
          <p className="auth-guard__signin-link">
            Already have an account?{" "}
            <button type="button" className="auth-guard__link" onClick={openSignIn}>
              Sign in
            </button>
          </p>
        </div>
      </div>

      {/* Auth Modals */}
      <SignInModal
        isOpen={activeModal === "signIn"}
        onClose={closeModal}
        onSwitchToSignUp={openSignUp}
        onForgotPassword={openForgotPassword}
      />

      <SignUpModal
        isOpen={activeModal === "signUp"}
        onClose={closeModal}
        onSwitchToSignIn={openSignIn}
      />

      <PasswordResetRequest
        isOpen={activeModal === "forgotPassword"}
        onClose={closeModal}
        onBackToSignIn={openSignIn}
      />
    </div>
  );
}

export default AuthGuard;
