# Development Cycle/Tooling
## Development Guide
_All developers are expected to follow the same outlined development cycle._

1. Any development must be done on a branch (not the main codeline). Branches should be named by `UANETID`/`TASK_NAME` (for example, I would could have a branch labeled `ajh269/throttleValve`.
2. Branches should be made with the intent to complete a single bite-sized issue, then merged into the main codeline. We do not want to create unmanaged branches that are each hundreds of commits ahead/behind each other (see [LE-Controls](https://github.com/Uakronauts/LE-Controls/branches)).
3. When creating a pull request, at least one other developer must perform a full review, and leave questions/comments/concerns wherever necessary.
    1. **As the PR creator**: you are responsible for properly documenting your PR with a description loosely following [the template](PR_GUIDE.md). Responding to and resolving all comments, and verifying a passing build before merging a PR. When merging your PR, select the "Squash" option, and delete your branch.
    2. **As a PR reviewer**: you are responsbile for reviewing the modified code, checking for functionality, style, or otherwise overall concerns.
4. Automated testing should be viewed as a mandatory action. Testing speeds up development immensely, and allows us to test all systems independently and together in a controlled manner. However, all tests should be meaningful and provide new functionality.
5. When applicable, break down components/functionality into minimal (reasonable) segments to allow for simple development, debugging, and testing.
6. This is a temendous amount of red tape, however this is industry standard and will create a highly reliable, easily debuggable, robust system. We have a comfortable amount of time to develop and polish features, and getting in these habits will adequately prepare you for the industry.

## Automated pipelines
Automated pipelines (or [Actions](https://github.com/Uakronauts/LCP-GUI/actions)) are an event that will run on certain events associated with the git repository. For example, at the time of writing this, we have an automated build/test pipeline that runs when creating a PR and pushing to main (see [Test](https://github.com/Uakronauts/LCP-GUI/actions/workflows/test.yml))

# Commands
## Install

Clone the repo and install dependencies:
**Having issues installing? See the ERB [debugging guide](https://github.com/electron-react-boilerplate/electron-react-boilerplate/issues/400)**
```bash
npm install
```

## Starting Development

Start the app in the `dev` environment:

```bash
npm start
```

## Running Tests

Build the project and run the designed testing library:

```bash
npm run test
```

## Packaging for Production

To package apps for the local platform:

```bash
npm run package
```

For more commands, see `scripts` in [package.json](package.json)

# Docs

- [React](https://react.dev/)
- [TypeScript](https://www.typescriptlang.org/)
- [FluentUI (library for basic components)](https://developer.microsoft.com/en-us/fluentui#/)
- [ESLint (style compiler)](https://eslint.org/docs/latest/)
- [Electron](https://www.electronjs.org/)
- [ERB Docs](https://electron-react-boilerplate.js.org/docs/installation)

