import React from 'react';
// Bridge wrapper to JS implementation to avoid TS type issues
// eslint-disable-next-line @typescript-eslint/no-var-requires
const Impl = require('./index.js').default;
export default function Navbar(props: any) {
  return <Impl {...props} />;
}
