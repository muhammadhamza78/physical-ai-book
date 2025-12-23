import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

interface PlatformTabsProps {
  children: React.ReactNode;
}

/**
 * Wrapper around Docusaurus Tabs for OS-specific instructions
 * Provides consistent platform tabs (Windows, macOS, Linux) across lessons
 */
export default function PlatformTabs({ children }: PlatformTabsProps): JSX.Element {
  return (
    <Tabs groupId="operating-systems">
      {children}
    </Tabs>
  );
}

// Re-export TabItem for convenience
export { TabItem };
