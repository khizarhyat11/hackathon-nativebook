/**
 * Custom Root component that wraps the entire Docusaurus site.
 * This injects the Chatbot component globally on all pages.
 */

import React from 'react';
import Root from '@theme-original/Root';
import { Chatbot } from '@site/src/components/Chatbot';

export default function RootWrapper(props: any): JSX.Element {
  return (
    <>
      <Root {...props} />
      <Chatbot />
    </>
  );
}
