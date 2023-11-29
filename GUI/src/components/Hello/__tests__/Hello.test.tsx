import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import { Hello } from '../Hello';

describe('App', () => {
  it('should render', () => {
    const { getByText } = render(<Hello />);

    expect(getByText('Hello!')).toBeInTheDocument();
  });
});
