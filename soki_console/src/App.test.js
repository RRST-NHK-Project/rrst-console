import { render, screen } from '@testing-library/react';

jest.mock('roslib', () => ({
  Ros: jest.fn().mockImplementation(() => ({
    on: jest.fn(),
    close: jest.fn(),
  })),
  Topic: jest.fn().mockImplementation(() => ({
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    publish: jest.fn(),
  })),
}), { virtual: true });

import App from './App';

test('renders soki console title', () => {
  render(<App />);
  expect(screen.getByText(/SOKI Console/i)).toBeInTheDocument();
});
