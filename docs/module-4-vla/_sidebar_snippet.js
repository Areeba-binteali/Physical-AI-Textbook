// docs/module-4-vla/_sidebar_snippet.js

/**
 * To add this module to the sidebar, import this snippet into your `sidebars.js` file
 * and spread it into the desired position in the `tutorialSidebar` array.
 *
 * @example
 * // sidebars.js
 * const module4 = require('./docs/module-4-vla/_sidebar_snippet.js');
 *
 * module.exports = {
 *   tutorialSidebar: [
 *     'intro',
 *     ...
 *     {
 *       type: 'category',
 *       label: 'Module 3: NVIDIA Isaac',
 *       items: [...]
 *     },
 *     ...module4, // <-- Like this
 *     ...
 *   ],
 * };
 */

module.exports = [
  {
    type: 'category',
    label: 'Module 4: Vision-Language-Action',
    link: {
      type: 'doc',
      id: 'module-4-vla/01-vla-philosophy'
    },
    items: [
      'module-4-vla/02-voice-to-text',
      'module-4-vla/03-language-to-plan',
      'module-4-vla/04-orchestrator',
      'module-4-vla/05-vision-grounding',
      'module-4-vla/06-capstone-project',
      'module-4-vla/00-review-and-next-steps',
    ]
  }
];
