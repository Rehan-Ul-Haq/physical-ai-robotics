import type { ReactNode } from "react";
import React, { useState, useEffect } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

// Typewriter effect hook
function useTypewriter(texts: string[], typingSpeed: number = 80, pauseDuration: number = 2000) {
  const [displayText, setDisplayText] = useState('');
  const [textIndex, setTextIndex] = useState(0);
  const [charIndex, setCharIndex] = useState(0);
  const [isDeleting, setIsDeleting] = useState(false);
  
  useEffect(() => {
    const currentText = texts[textIndex];
    
    const timeout = setTimeout(() => {
      if (!isDeleting) {
        if (charIndex < currentText.length) {
          setDisplayText(currentText.slice(0, charIndex + 1));
          setCharIndex(charIndex + 1);
        } else {
          setTimeout(() => setIsDeleting(true), pauseDuration);
        }
      } else {
        if (charIndex > 0) {
          setDisplayText(currentText.slice(0, charIndex - 1));
          setCharIndex(charIndex - 1);
        } else {
          setIsDeleting(false);
          setTextIndex((textIndex + 1) % texts.length);
        }
      }
    }, isDeleting ? typingSpeed / 2 : typingSpeed);
    
    return () => clearTimeout(timeout);
  }, [charIndex, isDeleting, textIndex, texts, typingSpeed, pauseDuration]);
  
  return displayText;
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  
  // Typewriter phrases
  const typewriterText = useTypewriter([
    'Build humanoid robots with AI',
    'Master ROS 2 & NVIDIA Isaac',
    'From simulation to reality',
    'Voice-controlled autonomy',
  ], 60, 2500);
  
  return (
    <header className={styles.heroBanner}>
      {/* Animated Background Elements */}
      <div className={styles.heroBackground}>
        <div className={styles.circuitGrid} />
        <div className={styles.glowOrb1} />
        <div className={styles.glowOrb2} />
        <div className={styles.scanLine} />
        
        {/* Floating Particles */}
        <div className={styles.particles}>
          {[...Array(20)].map((_, i) => (
            <div key={i} className={styles.particle} style={{
              left: `${Math.random() * 100}%`,
              animationDelay: `${Math.random() * 5}s`,
              animationDuration: `${8 + Math.random() * 7}s`,
            }} />
          ))}
        </div>
      </div>

      <div className="container">
        <div className={styles.heroContent}>
          {/* Left side - Robot Eye Hero Element */}
          <div className={styles.heroVisualSection}>
            {/* Large Robot Eye */}
            <div className={styles.robotEyeHero}>
              <div className={styles.robotEyeSocket}>
                <div className={styles.robotEyeBall} />
                <div className={styles.eyeReflection} />
              </div>
              <div className={styles.eyeRings}>
                <div className={styles.eyeRing1} />
                <div className={styles.eyeRing2} />
                <div className={styles.eyeRing3} />
              </div>
            </div>
            <div className={styles.eyeLabel}>ROBO AI</div>
            
            {/* Floating tech badges */}
            <div className={`${styles.floatingBadge} ${styles.floatingBadge1}`}>
              <span>ROS 2</span>
            </div>
            <div className={`${styles.floatingBadge} ${styles.floatingBadge2}`}>
              <span>Isaac</span>
            </div>
            <div className={`${styles.floatingBadge} ${styles.floatingBadge3}`}>
              <span>AI</span>
            </div>
          </div>

          {/* Right side - Content */}
          <div className={styles.heroTextContent}>
            {/* Status indicator */}
            <div className={styles.statusBar}>
              <span className={styles.statusDot} />
              <span className={styles.statusText}>SYSTEM ONLINE</span>
            </div>
            
            <Heading as="h1" className={styles.heroTitle}>
              <span className={styles.titleLine1}>Physical AI &</span>
              <span className={styles.titleLine2}>Humanoid Robotics</span>
            </Heading>
            
            {/* Typewriter tagline */}
            <div className={styles.typewriterContainer}>
              <span className={styles.typewriterPrefix}>{'>'} </span>
              <span className={styles.typewriterText}>{typewriterText}</span>
              <span className={styles.typewriterCursor}>|</span>
            </div>
            
            <p className={styles.heroSubtitle}>
              The complete guide to building <strong>embodied AI systems</strong> that 
              perceive, reason, and act in the physical world. From digital twins 
              to voice-controlled humanoids.
            </p>

            {/* Tech Stack Pills */}
            <div className={styles.techPills}>
              <span className={styles.techPill}>
                <span className={styles.pillDot} />ROS 2 Humble
              </span>
              <span className={styles.techPill}>
                <span className={styles.pillDot} />NVIDIA Isaac
              </span>
              <span className={styles.techPill}>
                <span className={styles.pillDot} />Gazebo Sim
              </span>
            </div>

            {/* CTA Buttons */}
            <div className={styles.heroButtons}>
              <Link
                className={styles.ctaButton}
                to="/docs/preface"
              >
                <span className={styles.ctaIcon}>
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <polygon points="5 3 19 12 5 21 5 3" />
                  </svg>
                </span>
                <span className={styles.ctaText}>Start Learning</span>
                <span className={styles.ctaArrow}>→</span>
              </Link>
              <Link
                className={styles.secondaryButton}
                href="https://github.com/panaversity"
                target="_blank"
                rel="noopener noreferrer"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
                </svg>
                <span>View on GitHub</span>
              </Link>
            </div>
          </div>
        </div>
      </div>
      
      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <div className={styles.scrollMouse}>
          <div className={styles.scrollWheel} />
        </div>
        <span>Explore Modules</span>
      </div>
    </header>
  );
}

// Course Modules Section
function ModulesSection() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>📚 Course Structure</div>
          <Heading as="h2" className={styles.sectionTitle}>
            Four Modules, One Goal:<br />
            <span className={styles.titleAccent}>Autonomous Humanoid Systems</span>
          </Heading>
          <p className={styles.sectionSubtitle}>
            Progressive learning from robot communication to conversational AI.
            Each module builds on the previous, culminating in a voice-controlled autonomous humanoid.
          </p>
        </div>

        <div className={styles.modulesGrid}>
          {/* Module 1 */}
          <div className={styles.moduleCard}>
            <div className={styles.moduleNumber}>01</div>
            <div className={styles.moduleIcon}>🔌</div>
            <h3 className={styles.moduleTitle}>The Robotic Nervous System</h3>
            <p className={styles.moduleDescription}>
              Master ROS 2 - the middleware powering modern robots. Build nodes, topics,
              and services to create distributed robotic systems.
            </p>
            <div className={styles.moduleMeta}>
              <span className={styles.moduleWeeks}>⏱️ Weeks 1-5</span>
              <span className={styles.moduleTech}>Python • rclpy • URDF</span>
            </div>
            <ul className={styles.moduleTopics}>
              <li>ROS 2 Architecture & Core Concepts</li>
              <li>Nodes, Topics, Services & Actions</li>
              <li>Building ROS 2 Packages with Python</li>
              <li>URDF Robot Description Format</li>
            </ul>
          </div>

          {/* Module 2 */}
          <div className={styles.moduleCard}>
            <div className={styles.moduleNumber}>02</div>
            <div className={styles.moduleIcon}>🌐</div>
            <h3 className={styles.moduleTitle}>The Digital Twin</h3>
            <p className={styles.moduleDescription}>
              Simulate physics-accurate environments in Gazebo. Test robots in virtual worlds
              before deploying to expensive hardware.
            </p>
            <div className={styles.moduleMeta}>
              <span className={styles.moduleWeeks}>⏱️ Weeks 6-7</span>
              <span className={styles.moduleTech}>Gazebo • Unity • SDF</span>
            </div>
            <ul className={styles.moduleTopics}>
              <li>Gazebo Simulation Environment Setup</li>
              <li>Physics Simulation: Gravity & Collisions</li>
              <li>Sensor Simulation: LiDAR, Cameras, IMUs</li>
              <li>Unity for High-Fidelity Visualization</li>
            </ul>
          </div>

          {/* Module 3 */}
          <div className={clsx(styles.moduleCard, styles.moduleCardFeatured)}>
            <div className={styles.moduleBadge}>⚡ Hardware Intensive</div>
            <div className={styles.moduleNumber}>03</div>
            <div className={styles.moduleIcon}>🧠</div>
            <h3 className={styles.moduleTitle}>The AI-Robot Brain</h3>
            <p className={styles.moduleDescription}>
              Advanced perception with NVIDIA Isaac. Photorealistic simulation,
              synthetic data generation, and hardware-accelerated SLAM.
            </p>
            <div className={styles.moduleMeta}>
              <span className={styles.moduleWeeks}>⏱️ Weeks 8-12</span>
              <span className={styles.moduleTech}>Isaac Sim • Isaac ROS • Nav2</span>
            </div>
            <ul className={styles.moduleTopics}>
              <li>NVIDIA Isaac Sim & Omniverse</li>
              <li>AI-Powered Perception & Manipulation</li>
              <li>Visual SLAM for Navigation</li>
              <li>Bipedal Locomotion & Balance Control</li>
            </ul>
          </div>

          {/* Module 4 */}
          <div className={clsx(styles.moduleCard, styles.moduleCardFeatured)}>
            <div className={styles.moduleBadge}>🎯 Capstone Project</div>
            <div className={styles.moduleNumber}>04</div>
            <div className={styles.moduleIcon}>🗣️</div>
            <h3 className={styles.moduleTitle}>Vision-Language-Action</h3>
            <p className={styles.moduleDescription}>
              Convergence of LLMs and robotics. Transform voice commands into robot actions.
              Build truly conversational humanoid systems.
            </p>
            <div className={styles.moduleMeta}>
              <span className={styles.moduleWeeks}>⏱️ Week 13</span>
              <span className={styles.moduleTech}>OpenAI • Whisper • GPT-4</span>
            </div>
            <ul className={styles.moduleTopics}>
              <li>Voice-to-Action with OpenAI Whisper</li>
              <li>LLM Cognitive Planning</li>
              <li>Multi-Modal Interaction (Voice, Vision, Gesture)</li>
              <li>Autonomous Task Execution</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

// Hardware Stack Section
function HardwareSection() {
  return (
    <section className={styles.hardwareSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>⚙️ Technical Requirements</div>
          <Heading as="h2" className={styles.sectionTitle}>
            The Physical AI Lab Stack
          </Heading>
          <p className={styles.sectionSubtitle}>
            This course sits at the intersection of three computationally demanding domains:
            <strong> Physics Simulation</strong>, <strong>Visual Perception</strong>, and <strong>Generative AI</strong>.
          </p>
        </div>

        <div className={styles.hardwareGrid}>
          {/* Workstation */}
          <div className={clsx(styles.hardwareCard, styles.hardwareCardPrimary)}>
            <div className={styles.hardwareIcon}>🖥️</div>
            <h3 className={styles.hardwareTitle}>Digital Twin Workstation</h3>
            <div className={styles.hardwareBadge}>Required</div>
            <p className={styles.hardwareDescription}>
              High-performance PC for Isaac Sim, Gazebo, and AI model training.
              RTX GPU required for ray-traced simulation.
            </p>
            <ul className={styles.hardwareSpecs}>
              <li><strong>GPU:</strong> NVIDIA RTX 4070 Ti+ (12GB VRAM min)</li>
              <li><strong>CPU:</strong> Intel i7 13th Gen / AMD Ryzen 9</li>
              <li><strong>RAM:</strong> 64GB DDR5 (32GB minimum)</li>
              <li><strong>OS:</strong> Ubuntu 22.04 LTS</li>
            </ul>
            <div className={styles.hardwareNote}>
              💡 ROS 2 Humble/Iron native to Linux. Windows requires WSL2.
            </div>
          </div>

          {/* Edge Kit */}
          <div className={styles.hardwareCard}>
            <div className={styles.hardwareIcon}>🔬</div>
            <h3 className={styles.hardwareTitle}>Physical AI Edge Kit</h3>
            <div className={styles.hardwareBadge}>Recommended</div>
            <p className={styles.hardwareDescription}>
              Deploy trained models to edge devices. Learn resource constraints
              and real-time inference optimization.
            </p>
            <ul className={styles.hardwareSpecs}>
              <li><strong>Brain:</strong> NVIDIA Jetson Orin Nano (8GB)</li>
              <li><strong>Eyes:</strong> Intel RealSense D435i (RGB-D + IMU)</li>
              <li><strong>Ears:</strong> ReSpeaker USB Mic Array</li>
              <li><strong>Total:</strong> ~$700 per kit</li>
            </ul>
            <div className={styles.hardwareNote}>
              🎯 Ideal for sim-to-real transfer and edge deployment
            </div>
          </div>

          {/* Robot */}
          <div className={styles.hardwareCard}>
            <div className={styles.hardwareIcon}>🤖</div>
            <h3 className={styles.hardwareTitle}>Robot Platform</h3>
            <div className={styles.hardwareBadge}>Optional</div>
            <p className={styles.hardwareDescription}>
              Physical robot for final deployment. Shared lab resource or
              simulation-only approach works too.
            </p>
            <ul className={styles.hardwareSpecs}>
              <li><strong>Budget:</strong> Unitree Go2 Edu (~$2,000)</li>
              <li><strong>Humanoid:</strong> Unitree G1 (~$16,000)</li>
              <li><strong>DIY:</strong> Hiwonder TonyPi (~$600)</li>
              <li><strong>Sim-Only:</strong> Isaac Sim virtual robots</li>
            </ul>
            <div className={styles.hardwareNote}>
              ☁️ Cloud option: AWS g5.2xlarge (~$200/quarter)
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Tech Stack Section
function TechStackSection() {
  return (
    <section className={styles.techSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>🛠️ Technologies</div>
          <Heading as="h2" className={styles.sectionTitle}>
            Industry-Standard Robotics Stack
          </Heading>
        </div>

        <div className={styles.techCategories}>
          {/* Middleware */}
          <div className={styles.techCategory}>
            <h3 className={styles.techCategoryTitle}>🔌 Middleware & Control</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>ROS 2 Humble/Iron</span>
              <span className={styles.techTag}>rclpy (Python)</span>
              <span className={styles.techTag}>Nav2</span>
              <span className={styles.techTag}>MoveIt 2</span>
            </div>
          </div>

          {/* Simulation */}
          <div className={styles.techCategory}>
            <h3 className={styles.techCategoryTitle}>🌐 Simulation & Physics</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>NVIDIA Isaac Sim</span>
              <span className={styles.techTag}>Gazebo Harmonic</span>
              <span className={styles.techTag}>Unity Robotics</span>
              <span className={styles.techTag}>URDF/SDF</span>
            </div>
          </div>

          {/* AI/ML */}
          <div className={styles.techCategory}>
            <h3 className={styles.techCategoryTitle}>🧠 AI & Perception</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>Isaac ROS (VSLAM)</span>
              <span className={styles.techTag}>OpenCV</span>
              <span className={styles.techTag}>PyTorch</span>
              <span className={styles.techTag}>OpenAI GPT-4</span>
              <span className={styles.techTag}>Whisper</span>
            </div>
          </div>

          {/* Hardware */}
          <div className={styles.techCategory}>
            <h3 className={styles.techCategoryTitle}>⚡ Hardware & Sensors</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>NVIDIA Jetson Orin</span>
              <span className={styles.techTag}>Intel RealSense</span>
              <span className={styles.techTag}>LiDAR</span>
              <span className={styles.techTag}>IMU (BNO055)</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Why Physical AI Section
function WhyPhysicalAISection() {
  return (
    <section className={styles.whySection}>
      <div className="container">
        <div className={styles.whyContent}>
          <div className={styles.whyLeft}>
            <div className={styles.sectionLabel}>💡 The Future is Embodied</div>
            <Heading as="h2" className={styles.sectionTitle}>
              Why Humanoid Robots?
            </Heading>
            <p className={styles.whyText}>
              Humanoid robots are poised to excel in our human-centered world because
              <strong> they share our physical form</strong>. They can navigate stairs,
              open doors, manipulate tools, and interact in spaces designed for us.
            </p>
            <p className={styles.whyText}>
              This represents a <strong>fundamental transition</strong> from AI models
              confined to digital environments to <strong>embodied intelligence</strong> that
              operates in physical space—understanding gravity, friction, momentum, and social interaction.
            </p>
            <ul className={styles.whyList}>
              <li>✅ Train on abundant human demonstration data</li>
              <li>✅ Navigate human-built environments naturally</li>
              <li>✅ Use existing tools without modification</li>
              <li>✅ Understand physics through embodied experience</li>
            </ul>
          </div>
          <div className={styles.whyRight}>
            <div className={styles.whyCard}>
              <h3 className={styles.whyCardTitle}>🎯 Capstone Project</h3>
              <p className={styles.whyCardDescription}>
                Build an <strong>Autonomous Conversational Humanoid</strong> that:
              </p>
              <ol className={styles.whyCardSteps}>
                <li>Receives voice command: <em>"Clean the table"</em></li>
                <li>Uses LLM to plan action sequence</li>
                <li>Navigates obstacles using VSLAM</li>
                <li>Identifies objects with computer vision</li>
                <li>Manipulates objects with humanoid hands</li>
                <li>Reports completion status via speech</li>
              </ol>
              <div className={styles.whyCardCTA}>
                <Link to="/docs/00-preface" className={styles.inlineLink}>
                  Start Building →
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// AI-Driven Development CTA
function AIApproachSection() {
  return (
    <section className={styles.approachSection}>
      <div className="container">
        <div className={styles.approachBox}>
          <div className={styles.approachIcon}>🚀</div>
          <div className={styles.approachContent}>
            <h2 className={styles.approachTitle}>
              Built the AI-Driven Way
            </h2>
            <p className={styles.approachDescription}>
              This course teaches robotics development using <strong>AI as your co-creator</strong>.
              Generate ROS 2 packages, URDF robot models, and navigation controllers through
              intelligent collaboration. Focus on architecture and intent—let AI handle boilerplate.
            </p>
            <div className={styles.approachBenefits}>
              <div className={styles.benefit}>
                <span className={styles.benefitIcon}>⚡</span>
                <span className={styles.benefitText}>10x faster prototyping</span>
              </div>
              <div className={styles.benefit}>
                <span className={styles.benefitIcon}>🧠</span>
                <span className={styles.benefitText}>Focus on system design</span>
              </div>
              <div className={styles.benefit}>
                <span className={styles.benefitIcon}>🎯</span>
                <span className={styles.benefitText}>Production-ready code</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master ROS 2, NVIDIA Isaac Sim, and embodied intelligence. Build humanoid robots using AI-driven development from digital brains to physical bodies."
    >
      <HomepageHeader />
      <ModulesSection />
      <HardwareSection />
      <TechStackSection />
      <WhyPhysicalAISection />
      <AIApproachSection />
    </Layout>
  );
}
