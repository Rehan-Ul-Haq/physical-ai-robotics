/**
 * UserMenu Component
 *
 * Dropdown menu for authenticated users in the navbar.
 * Implements T082-T084, T096, T098 from the task list.
 *
 * Features:
 * - User avatar and name display
 * - Dropdown menu with Profile, Settings, Sign Out options
 * - Sign out functionality with redirect
 * - Keyboard navigation support
 */
import React, { useState, useRef, useEffect } from "react";
import { signOut, useSession } from "@/lib/auth";

interface UserMenuProps {
  /** Optional class name for styling */
  className?: string;
}

export function UserMenu({ className = "" }: UserMenuProps) {
  const { data: session } = useSession();
  const [isOpen, setIsOpen] = useState(false);
  const [isSigningOut, setIsSigningOut] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);
  const triggerRef = useRef<HTMLButtonElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener("mousedown", handleClickOutside);
      return () => document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [isOpen]);

  // Close menu on escape key
  useEffect(() => {
    function handleEscape(event: KeyboardEvent) {
      if (event.key === "Escape" && isOpen) {
        setIsOpen(false);
        triggerRef.current?.focus();
      }
    }

    document.addEventListener("keydown", handleEscape);
    return () => document.removeEventListener("keydown", handleEscape);
  }, [isOpen]);

  /**
   * Handle sign out
   */
  const handleSignOut = async () => {
    setIsSigningOut(true);
    try {
      await signOut();
      // Redirect to homepage after sign out (T098)
      window.location.href = "/physical-ai-robotics/";
    } catch (err) {
      console.error("[UserMenu] Sign out failed:", err);
      setIsSigningOut(false);
      // Optionally show error toast
    }
  };

  /**
   * Toggle dropdown menu
   */
  const toggleMenu = () => {
    setIsOpen((prev) => !prev);
  };

  /**
   * Handle menu item click
   */
  const handleMenuItemClick = (action: () => void) => {
    setIsOpen(false);
    action();
  };

  if (!session) {
    return null;
  }

  const { user } = session;
  const displayName = user.name || user.email?.split("@")[0] || "User";
  const avatarUrl = user.image || null;
  const initials = displayName
    .split(" ")
    .map((n) => n[0])
    .join("")
    .toUpperCase()
    .slice(0, 2);

  return (
    <div className={`user-menu ${className}`} ref={menuRef}>
      {/* Trigger button */}
      <button
        ref={triggerRef}
        type="button"
        className="user-menu__trigger"
        onClick={toggleMenu}
        aria-expanded={isOpen}
        aria-haspopup="menu"
        aria-label={`User menu for ${displayName}`}
      >
        {avatarUrl ? (
          <img
            src={avatarUrl}
            alt={displayName}
            className="user-menu__avatar"
          />
        ) : (
          <span className="user-menu__avatar user-menu__avatar--initials">
            {initials}
          </span>
        )}
        <span className="user-menu__name">{displayName}</span>
        <ChevronIcon className="user-menu__chevron" isOpen={isOpen} />
      </button>

      {/* Dropdown menu */}
      {isOpen && (
        <div className="user-menu__dropdown" role="menu">
          {/* User info header */}
          <div className="user-menu__header">
            <span className="user-menu__header-name">{displayName}</span>
            <span className="user-menu__header-email">{user.email}</span>
          </div>

          <div className="user-menu__divider" role="separator" />

          {/* Menu items */}
          <a
            href="/profile"
            className="user-menu__item"
            role="menuitem"
            onClick={() => setIsOpen(false)}
          >
            <ProfileIcon className="user-menu__item-icon" />
            Profile
          </a>

          <a
            href="/settings/account"
            className="user-menu__item"
            role="menuitem"
            onClick={() => setIsOpen(false)}
          >
            <SettingsIcon className="user-menu__item-icon" />
            Settings
          </a>

          <div className="user-menu__divider" role="separator" />

          <button
            type="button"
            className="user-menu__item user-menu__item--danger"
            role="menuitem"
            onClick={() => handleMenuItemClick(handleSignOut)}
            disabled={isSigningOut}
          >
            <SignOutIcon className="user-menu__item-icon" />
            {isSigningOut ? "Signing out..." : "Sign Out"}
          </button>
        </div>
      )}
    </div>
  );
}

/**
 * Chevron icon for dropdown indicator
 */
function ChevronIcon({ className, isOpen }: { className?: string; isOpen: boolean }) {
  return (
    <svg
      className={className}
      width="16"
      height="16"
      viewBox="0 0 16 16"
      fill="none"
      style={{ transform: isOpen ? "rotate(180deg)" : "rotate(0deg)", transition: "transform 0.2s" }}
      aria-hidden="true"
    >
      <path
        d="M4 6L8 10L12 6"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

/**
 * Profile icon
 */
function ProfileIcon({ className }: { className?: string }) {
  return (
    <svg className={className} width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <path
        d="M8 8C9.65685 8 11 6.65685 11 5C11 3.34315 9.65685 2 8 2C6.34315 2 5 3.34315 5 5C5 6.65685 6.34315 8 8 8Z"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M2 14C2 11.7909 4.68629 10 8 10C11.3137 10 14 11.7909 14 14"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

/**
 * Settings icon
 */
function SettingsIcon({ className }: { className?: string }) {
  return (
    <svg className={className} width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <path
        d="M8 10C9.10457 10 10 9.10457 10 8C10 6.89543 9.10457 6 8 6C6.89543 6 6 6.89543 6 8C6 9.10457 6.89543 10 8 10Z"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M13.0607 10.0607C13.1706 9.79201 13.2045 9.49657 13.1582 9.20853C13.1119 8.92049 12.9872 8.65136 12.7986 8.43C13.1234 8.08533 13.3333 7.64467 13.3333 7.16667V6.83333C13.3333 6.35533 13.1234 5.91467 12.7986 5.57C12.9872 5.34864 13.1119 5.07951 13.1582 4.79147C13.2045 4.50343 13.1706 4.20799 13.0607 3.93933C12.9508 3.67067 12.7694 3.43932 12.5371 3.27093C12.3048 3.10254 12.0307 3.00354 11.7453 2.98467L11.6667 2.98L11.3333 3C10.8593 3 10.4173 3.21 10.0727 3.53467C9.85132 3.34608 9.58219 3.22142 9.29415 3.17515C9.00611 3.12889 8.71067 3.16274 8.442 3.27267C8.17333 3.38259 7.942 3.56399 7.77361 3.79628C7.60522 4.02856 7.50621 4.30268 7.48734 4.58801L7.48267 4.66667L7.50267 5C7.50267 5.474 7.29267 5.916 6.968 6.26067C6.746 6.07141 6.47613 5.94648 6.18729 5.90036C5.89845 5.85424 5.60224 5.88862 5.33301 6.00001C5.06378 6.11139 4.83228 6.29532 4.66479 6.53085C4.4973 6.76639 4.40052 7.04431 4.38534 7.33334L4.38267 7.412L4.404 7.74667C4.404 8.22067 4.19334 8.66267 3.86867 9.00734C3.64731 8.81875 3.37818 8.69408 3.09014 8.64782C2.80211 8.60155 2.50666 8.6354 2.238 8.74534C1.96933 8.85526 1.738 9.03665 1.56961 9.26894C1.40122 9.50122 1.30222 9.77534 1.28334 10.0607L1.28 10.1393L1.30067 10.4727C1.30067 10.9467 1.51134 11.3887 1.836 11.7333C1.62741 11.9547 1.50274 12.2239 1.45648 12.5119C1.41022 12.7999 1.44406 13.0954 1.554 13.364C1.66393 13.6327 1.84532 13.864 2.07761 14.0324C2.30989 14.2008 2.58401 14.2998 2.86934 14.3187L2.948 14.322L3.28134 14.3007C3.75534 14.3007 4.19734 14.5107 4.542 14.8353C4.76336 14.6467 5.03249 14.5221 5.32053 14.4758C5.60857 14.4295 5.90401 14.4634 6.17267 14.5733C6.44133 14.6833 6.67268 14.8647 6.84107 15.0969C7.00946 15.3292 7.10847 15.6034 7.12734 15.8887L7.13067 15.9673L7.10934 16.3007C7.10934 16.7747 7.31934 17.2167 7.644 17.5613C7.86536 17.3727 8.13449 17.2481 8.42253 17.2018C8.71057 17.1555 9.00601 17.1894 9.27467 17.2993C9.54333 17.4093 9.77468 17.5907 9.94307 17.8229C10.1115 18.0552 10.2105 18.3293 10.2293 18.6147L10.232 18.6933L10.2107 19.0267"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

/**
 * Sign out icon
 */
function SignOutIcon({ className }: { className?: string }) {
  return (
    <svg className={className} width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <path
        d="M6 14H3.33333C2.97971 14 2.64057 13.8595 2.39052 13.6095C2.14048 13.3594 2 13.0203 2 12.6667V3.33333C2 2.97971 2.14048 2.64057 2.39052 2.39052C2.64057 2.14048 2.97971 2 3.33333 2H6"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M10.6667 11.3333L14 8L10.6667 4.66667"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M14 8H6"
        stroke="currentColor"
        strokeWidth="1.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

export default UserMenu;
