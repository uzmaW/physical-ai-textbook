/**
 * Sidebar navigation for Physical AI & Humanoid Robotics textbook.
 * 13-week structure with visual separators.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '📖 Introduction',
    },

    // Weeks 1-2: Foundations
    {
      type: 'category',
      label: '🎯 Foundations',
      collapsed: false,
      items: [
        { type: 'doc', id: 'week-01', label: 'Physical AI & Embodied Intelligence' },
        { type: 'doc', id: 'week-02', label: 'Realistic Interaction & Emotional AI' },
      ],
    },

    // Weeks 3-5: ROS 2
    {
      type: 'category',
      label: '⚙️ ROS 2 & Hardware',
      collapsed: true,
      items: [
        { type: 'doc', id: 'week-03', label: 'ROS 2 Fundamentals' },
        { type: 'doc', id: 'week-04', label: 'Services, Actions & Launch Files' },
        { type: 'doc', id: 'week-05', label: 'Transforms, Control & Multi-Robot' },
      ],
    },

    // Weeks 6-7: Simulation
    {
      type: 'category',
      label: '🎮 Simulation',
      collapsed: true,
      items: [
        { type: 'doc', id: 'week-06', label: 'Gazebo & Unity Simulation' },
        { type: 'doc', id: 'week-07', label: 'Domain Randomization & Sim-to-Real' },
      ],
    },

    // Weeks 8-9: GPU AI
    {
      type: 'category',
      label: '🤖 GPU-Accelerated AI',
      collapsed: true,
      items: [
        { type: 'doc', id: 'week-08', label: 'NVIDIA Isaac Sim & Isaac ROS' },
        { type: 'doc', id: 'week-09', label: 'Deep Reinforcement Learning' },
      ],
    },

    // Weeks 10-11: Advanced AI
    {
      type: 'category',
      label: '🧠 Advanced AI & Control',
      collapsed: true,
      items: [
        { type: 'doc', id: 'week-10', label: 'Advanced RL & Imitation Learning' },
        { type: 'doc', id: 'week-11', label: 'Humanoid Kinematics & Locomotion' },
      ],
    },

    // Week 12: Applications
    {
      type: 'category',
      label: '🌍 Real-World Applications',
      collapsed: true,
      items: [
        { type: 'doc', id: 'week-12', label: 'VLA Models & Industry Applications' },
      ],
    },

    // Week 13: Capstone (Special styling - red/bold)
    {
      type: 'category',
      label: '🎓 Capstone Project',
      collapsed: false,
      items: [
        { type: 'doc', id: 'week-13', label: 'Voice-Commanded Autonomous Humanoid' },
      ],
    },

    {
      type: 'doc',
      id: 'bibliography',
      label: '📚 Bibliography',
    },
    {
      type: 'doc',
      id: 'glossary',
      label: '📝 Glossary',
    },
  ],
};

export default sidebars;
