import React from 'react';

interface HardwareItem {
  name: string;
  quantity: number;
  price: string;
  alternatives?: string;
}

interface HardwareListProps {
  items: HardwareItem[];
}

export default function HardwareList({ items }: HardwareListProps): JSX.Element {
  return (
    <table className="hardware-table">
      <thead>
        <tr>
          <th>Component</th>
          <th>Quantity</th>
          <th>Est. Price</th>
          <th>Alternatives</th>
        </tr>
      </thead>
      <tbody>
        {items.map((item, index) => (
          <tr key={index}>
            <td>{item.name}</td>
            <td>{item.quantity}</td>
            <td>{item.price}</td>
            <td>{item.alternatives || 'N/A'}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}
