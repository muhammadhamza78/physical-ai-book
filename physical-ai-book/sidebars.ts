import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Welcome',
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI',
      collapsed: false,
      link: {
        type: 'doc',
        id: 'module-01/index',
      },
      items: [
        'module-01/chapter-01-understanding-physical-ai',
        'module-01/chapter-02-sensing-the-world',
        'module-01/chapter-03-acting-on-information',
        'module-01/module-01-project',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins & Simulation',
      collapsed: false,
      link: {
        type: 'doc',
        id: 'module-02/index',
      },
      items: [
        'module-02/chapter-01-digital-twins',
        'module-02/chapter-02-gazebo-physics',
        'module-02/chapter-03-unity-rendering',
        'module-02/chapter-04-sensor-simulation',
        'module-02/installation-guide',
        'module-02/module-02-project',
        'module-02/troubleshooting',
        'module-02/glossary',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception & Navigation',
      collapsed: false,
      link: {
        type: 'doc',
        id: 'module-03/index',
      },
      items: [
        'module-03/chapter-01-introduction',
        'module-03/chapter-02-synthetic-data',
        'module-03/chapter-03-isaac-ros-vslam',
        'module-03/chapter-04-nav2-navigation',
        'module-03/module-03-project',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Task Planning & Execution',
      collapsed: false,
      link: {
        type: 'doc',
        id: 'module-04/index',
      },
      items: [
        'module-04/chapter-01-introduction',
        'module-04/chapter-02-plan-tasks',
        'module-04/module-04-project',
      ],
    },
    {
      type: 'doc',
      id: 'glossary',
      label: 'Glossary',
    },
  ],
};
export default sidebars;
