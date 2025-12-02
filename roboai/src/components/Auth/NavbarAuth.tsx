/**
 * NavbarAuth Component
 *
 * Authentication UI for the Docusaurus Navbar.
 * Shows Sign In/Sign Up buttons for anonymous users,
 * or UserMenu for authenticated users.
 *
 * Implements T035, T048, T083 from the task list.
 */
import React, { useState } from "react";
import { useSession } from "@/lib/auth";
import { SignInModal } from "@/components/Auth/SignInModal";
import { SignUpModal } from "@/components/Auth/SignUpModal";
import { PasswordResetRequest } from "@/components/Auth/PasswordResetRequest";
import { UserMenu } from "@/components/Auth/UserMenu";
import "@/components/Auth/Auth.css";

type ModalState = "none" | "signIn" | "signUp" | "forgotPassword";

export function NavbarAuth(): React.ReactElement {
  const { data: session, isPending } = useSession();
  const [activeModal, setActiveModal] = useState<ModalState>("none");

  /**
   * Handle modal switching
   */
  const openSignIn = () => setActiveModal("signIn");
  const openSignUp = () => setActiveModal("signUp");
  const openForgotPassword = () => setActiveModal("forgotPassword");
  const closeModal = () => setActiveModal("none");

  // Loading state - show skeleton
  if (isPending) {
    return (
      <div className="navbar-auth navbar-auth--loading">
        <div className="navbar-auth__skeleton" />
      </div>
    );
  }

  // Authenticated state - show user menu
  if (session) {
    return (
      <div className="navbar-auth">
        <UserMenu />
      </div>
    );
  }

  // Anonymous state - show auth buttons
  return (
    <div className="navbar-auth">
      <button
        type="button"
        className="navbar-auth__btn navbar-auth__btn--secondary"
        onClick={openSignIn}
      >
        Sign In
      </button>
      <button
        type="button"
        className="navbar-auth__btn navbar-auth__btn--primary"
        onClick={openSignUp}
      >
        Sign Up
      </button>

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

export default NavbarAuth;
