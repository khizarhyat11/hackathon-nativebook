import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HeroBanner() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master the intersection of artificial intelligence and physical robotics. 
            Build intelligent systems from ROS 2 to voice-controlled autonomous humanoids.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning →
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/category/robotic-nervous-system">
              View Curriculum
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

type ModuleItem = {
  number: string;
  title: string;
  description: string;
  deliverable: string;
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    number: '01',
    title: 'The Robotic Nervous System',
    description: 'ROS 2 Architecture, Python Bridging, URDF',
    deliverable: 'Hello Robot node + Bipedal URDF model',
    link: '/docs/category/robotic-nervous-system',
  },
  {
    number: '02',
    title: 'The Digital Twin',
    description: 'Gazebo Physics, Unity Rendering, Sensor Simulation',
    deliverable: 'Simulation with obstacle sensing',
    link: '/docs/category/digital-twin',
  },
  {
    number: '03',
    title: 'The AI-Robot Brain',
    description: 'Isaac Sim, Visual SLAM, Nav2 Navigation',
    deliverable: 'Robot that maps and navigates A→B',
    link: '/docs/category/ai-robot-brain',
  },
  {
    number: '04',
    title: 'Vision-Language-Action',
    description: 'Whisper Voice, LLM Planning, Full Integration',
    deliverable: 'The Autonomous Humanoid (Capstone)',
    link: '/docs/category/vision-language-action',
  },
];

function ModuleCard({number, title, description, deliverable, link}: ModuleItem) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleNumber}>Module {number}</div>
      <Heading as="h3">{title}</Heading>
      <p className={styles.moduleDescription}>{description}</p>
      <div className={styles.moduleDeliverable}>
        <strong>📦 Deliverable:</strong> {deliverable}
      </div>
      <Link className={styles.moduleLink} to={link}>
        Start Module →
      </Link>
    </div>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">The Curriculum</Heading>
          <p>Four modules taking you from ROS 2 basics to a voice-controlled autonomous humanoid</p>
        </div>
        <div className={styles.moduleGrid}>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStackSection() {
  const technologies = [
    { name: 'ROS 2 Humble', icon: '🤖' },
    { name: 'Gazebo', icon: '🌐' },
    { name: 'NVIDIA Isaac', icon: '🧠' },
    { name: 'OpenAI Whisper', icon: '🎤' },
    { name: 'GPT-4 / LLMs', icon: '💬' },
    { name: 'Python', icon: '🐍' },
  ];

  return (
    <section className={styles.techStack}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">Technology Stack</Heading>
          <p>Industry-standard tools for Physical AI development</p>
        </div>
        <div className={styles.techGrid}>
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techItem}>
              <span className={styles.techIcon}>{tech.icon}</span>
              <span>{tech.name}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2">Ready to Build the Future?</Heading>
          <p>Start your journey from ROS 2 fundamentals to autonomous humanoid robots.</p>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Begin Your Journey →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactElement {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="The AI-Native Textbook for Physical AI & Humanoid Robotics. From ROS 2 to voice-controlled autonomous humanoids.">
      <HeroBanner />
      <main>
        <ModulesSection />
        <TechStackSection />
        <CTASection />
      </main>
    </Layout>
  );
}
