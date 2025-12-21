import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  curriculumSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 01: The Robotic Nervous System',
      link: {
        type: 'generated-index',
        title: 'The Robotic Nervous System (ROS 2)',
        description: 'Establish the middleware foundation for robot control.',
        slug: '/category/robotic-nervous-system',
      },
      items: [
        'robotic-nervous-system/ros2-architecture',
        'robotic-nervous-system/python-bridging',
        'robotic-nervous-system/urdf-anatomy',
      ],
    },
    {
      type: 'category',
      label: 'Module 02: The Digital Twin',
      link: {
        type: 'generated-index',
        title: 'The Digital Twin (Gazebo & Unity)',
        description: 'Master physics simulation and high-fidelity environment building.',
        slug: '/category/digital-twin',
      },
      items: [
        'digital-twin/physics-engines',
        'digital-twin/unity-rendering',
        'digital-twin/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 03: The AI-Robot Brain',
      link: {
        type: 'generated-index',
        title: 'The AI-Robot Brain (NVIDIA Isaac)',
        description: 'Implement advanced perception and VSLAM.',
        slug: '/category/ai-robot-brain',
      },
      items: [
        'ai-robot-brain/isaac-sim',
        'ai-robot-brain/visual-slam',
        'ai-robot-brain/nav2-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 04: Vision-Language-Action',
      link: {
        type: 'generated-index',
        title: 'Vision-Language-Action (VLA)',
        description: 'The convergence of LLMs and Physical Robotics.',
        slug: '/category/vision-language-action',
      },
      items: [
        'vision-language-action/voice-pipeline',
        'vision-language-action/cognitive-logic',
        'vision-language-action/capstone',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
