import React from 'react';
import Link from '@docusaurus/Link';

interface LessonCardProps {
  title: string;
  description: string;
  duration: string;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  link: string;
}

export default function LessonCard({
  title,
  description,
  duration,
  difficulty,
  link,
}: LessonCardProps): JSX.Element {
  const getDifficultyColor = (level: string) => {
    switch (level) {
      case 'Beginner':
        return '#28a745';
      case 'Intermediate':
        return '#ffc107';
      case 'Advanced':
        return '#dc3545';
      default:
        return '#6c757d';
    }
  };

  return (
    <Link to={link} className="lesson-card" style={{ textDecoration: 'none', color: 'inherit' }}>
      <h3>{title}</h3>
      <p>{description}</p>
      <div className="lesson-meta">
        <span>⏱️ {duration}</span>
        <span style={{
          color: getDifficultyColor(difficulty),
          fontWeight: 500
        }}>
          📊 {difficulty}
        </span>
      </div>
    </Link>
  );
}
