import React, {type ReactNode, useEffect, useRef} from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { NavbarAuth } from '@/components/Auth/NavbarAuth';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): ReactNode {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Insert auth before the theme toggle in navbar__items--right
    const container = containerRef.current;
    if (!container) return;

    const rightItems = container.querySelector('.navbar__items--right');
    const authContainer = container.querySelector('[data-navbar-auth]');
    
    if (rightItems && authContainer) {
      // Find the color mode toggle (usually the last element)
      const children = Array.from(rightItems.children);
      const toggleContainer = children.find(child => 
        child.querySelector('.clean-btn') || 
        child.querySelector('[class*="toggle"]') ||
        child.querySelector('[class*="colorMode"]')
      );
      
      if (toggleContainer) {
        // Insert auth before the toggle
        rightItems.insertBefore(authContainer, toggleContainer);
      } else {
        // If no toggle found, just append
        rightItems.appendChild(authContainer);
      }
    }
  }, []);

  return (
    <div ref={containerRef} style={{ display: 'contents' }}>
      <Content {...props} />
      <div data-navbar-auth="" style={{ display: 'flex', alignItems: 'center' }}>
        <NavbarAuth />
      </div>
    </div>
  );
}
