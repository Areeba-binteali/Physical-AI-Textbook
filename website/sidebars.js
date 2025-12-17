// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  bookSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/introduction',
        'module-1-ros2/nodes-and-topics',
        'module-1-ros2/first-ros-program',
        'module-1-ros2/services-and-actions',
        'module-1-ros2/building-a-service',
        'module-1-ros2/intro-to-urdf',
        'module-1-ros2/bridging-ai-to-ros',
        'module-1-ros2/review-and-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/introduction-to-digital-twins',
        'module-2-digital-twin/gazebo-architecture-workflow',
        'module-2-digital-twin/physics-simulation',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/gazebo-vs-unity',
        'module-2-digital-twin/review-and-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim',
      items: [
        'module-3-isaac/00-review-and-next-steps',
        'module-3-isaac/01-isaac-overview',
        'module-3-isaac/02-synthetic-data',
        'module-3-isaac/03-isaac-ros-perception',
        'module-3-isaac/04-vslam-localization',
        'module-3-isaac/05-navigation-planning',
        'module-3-isaac/06-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/review-and-next-steps',
        'module-4-vla/vla-philosophy',
        'module-4-vla/voice-to-text',
        'module-4-vla/language-to-plan',
        'module-4-vla/orchestrator',
        'module-4-vla/vision-grounding',
        'module-4-vla/capstone-project',
      ],
    },
  ],
};

export default sidebars;
