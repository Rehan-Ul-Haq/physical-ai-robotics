import React from 'react';
import Navbar from '@theme-original/Navbar';
import type { WrapperProps } from '@docusaurus/types';
import styles from './styles.module.css';

type Props = WrapperProps<typeof Navbar>;

export default function NavbarWrapper(props: Props): React.ReactElement {
  return (
    <div className={styles.navbarContainer}>
      {/* Animated Circuit Grid Background */}
      <div className={styles.circuitGrid}>
        <div className={styles.gridLine} />
        <div className={styles.gridLine} />
        <div className={styles.gridLine} />
        <div className={styles.gridLine} />
        <div className={styles.gridLine} />
      </div>

      {/* Glowing Energy Bar - Top */}
      <div className={styles.energyBar}>
        <div className={styles.energyPulse} />
      </div>

      {/* Left Side - AI Brain Icon */}
      <div className={styles.leftDecoration}>
        <div className={styles.brainIcon}>
          <svg viewBox="0 0 32 32" className={styles.brainSvg}>
            {/* Brain outline */}
            <path 
              d="M16 4C10 4 6 8 6 14c0 4 2 7 5 9v3c0 1 1 2 2 2h6c1 0 2-1 2-2v-3c3-2 5-5 5-9 0-6-4-10-10-10z" 
              className={styles.brainPath}
            />
            {/* Neural connections inside */}
            <circle cx="12" cy="12" r="1.5" className={styles.neuron} />
            <circle cx="16" cy="10" r="1.5" className={styles.neuron} />
            <circle cx="20" cy="12" r="1.5" className={styles.neuron} />
            <circle cx="14" cy="16" r="1.5" className={styles.neuron} />
            <circle cx="18" cy="16" r="1.5" className={styles.neuron} />
            <line x1="12" y1="12" x2="16" y2="10" className={styles.synapse} />
            <line x1="16" y1="10" x2="20" y2="12" className={styles.synapse} />
            <line x1="12" y1="12" x2="14" y2="16" className={styles.synapse} />
            <line x1="20" y1="12" x2="18" y2="16" className={styles.synapse} />
            <line x1="14" y1="16" x2="18" y2="16" className={styles.synapse} />
          </svg>
        </div>
        <span className={styles.aiLabel}>AI</span>
      </div>

      {/* Right Side - Status Badge */}
      <div className={styles.rightDecoration}>
        <div className={styles.statusBadge}>
          <span className={styles.statusPulse} />
          <span className={styles.statusLabel}>SYSTEM ACTIVE</span>
        </div>
      </div>

      {/* Floating Particles */}
      <div className={styles.particles}>
        <span className={styles.particle} />
        <span className={styles.particle} />
        <span className={styles.particle} />
        <span className={styles.particle} />
        <span className={styles.particle} />
      </div>

      <Navbar {...props} />
      
      {/* Bottom Circuit Trace */}
      <div className={styles.bottomTrace}>
        <svg viewBox="0 0 1200 8" preserveAspectRatio="none" className={styles.traceSvg}>
          <defs>
            <linearGradient id="traceGradient" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="transparent" />
              <stop offset="20%" stopColor="var(--trace-color)" />
              <stop offset="50%" stopColor="var(--trace-glow)" />
              <stop offset="80%" stopColor="var(--trace-color)" />
              <stop offset="100%" stopColor="transparent" />
            </linearGradient>
          </defs>
          <rect x="0" y="2" width="1200" height="4" fill="url(#traceGradient)" rx="2" />
          {/* Moving highlight */}
          <rect className={styles.traceHighlight} x="0" y="1" width="120" height="6" rx="3" />
        </svg>
      </div>
    </div>
  );
}
