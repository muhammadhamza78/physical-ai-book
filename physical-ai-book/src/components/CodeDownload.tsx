import React from 'react';
import Link from '@docusaurus/Link';

interface CodeDownloadProps {
  filename: string;
  filepath: string;
  size: string;
}

export default function CodeDownload({
  filename,
  filepath,
  size,
}: CodeDownloadProps): JSX.Element {
  return (
    <div style={{ marginTop: '1rem', marginBottom: '1rem' }}>
      <Link to={filepath} className="code-download-btn" download>
        <span>⬇️ Download {filename}</span>
        <span style={{ fontSize: '0.875rem', opacity: 0.9 }}>({size})</span>
      </Link>
    </div>
  );
}
