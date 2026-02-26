# pyrebuilt

The Warriors' robot code for FRC REBUILT 2026

## Reference Material

- [Python tutorial][python]
- [GitHub tutorial][github]
- [FRC robot Python package][robotpy]
  - [Python MagicBot framework][magicbot]


[python]: https://www.tutorialspoint.com/python/index.htm
[github]: https://docs.github.com/en/get-started/start-your-journey/hello-world
[robotpy]: https://robotpy.readthedocs.io/en/stable/
[magicbot]: https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html


## Setup

### Preconditions

#### GitHub

- Create an account here on GitHub: https://github.com/signup

- Download a copy of GitHub Desktop: https://desktop.github.com/download/

#### Coding Environment

- Download VS Code: https://code.visualstudio.com/download

#### Download code

- Use GitHub Desktop to clone this repository (`File->Clone repository...`)
  - If you have been added to the `frc1880` team the repo should automatically appear in the list.
  - If not, you can enter the URL. *Make sure to let one of the mentors know your GitHub username so you can be added later*.
  - Remember where it downloaded it to (by default `C:\Users\<your-username>\Documents\GitHub`).
- Open VS Code
   - `File->Open Folder...`
   - Select the folder that GitHub Desktop cloned this repository into.
   - Start a terminal to perform the commands needed in the next sections:
     - `Terminal->New Terminal Window`
     - Make sure the prompt shows you in the directory with this code in it (for example `C:\Users\james\Documents\GitHub\pyrebuilt`)
       - If not, change to that directory: `cd \Users\<your-username>\Documents\GitHub\pyrebuilt`

### Install dependencies

We use `uv` to manage our dependencies in our development environments.
This includes the Python version, and any Python packages such as `wpilib`.

Install `uv` by following the [`uv` docs](https://docs.astral.sh/uv/).

After installing `uv`, download the roboRIO dependencies.

```sh
uv run robotpy sync --no-install
```

`uv run` will automatically create a virtual environment and install our dependencies for local development.

### pre-commit

[pre-commit][] is configured to run our formatters and linters.
These are enforced for all code committed to this project.

We recommend using [prek][] to run our pre-commit hooks.
See the [prek installation docs][] to install prek.

You can then set up the pre-commit hooks to run on commit:

```sh
prek install
```

[pre-commit]: https://pre-commit.com
[prek]: https://prek.j178.dev
[prek installation docs]: https://prek.j178.dev/installation/

### IDE setup

#### Visual Studio Code

1. Install the workspace's recommended extensions.
2. Copy `.vscode/settings.default.json` to `.vscode/settings.json`.

   ```sh
   cp .vscode/settings.default.json .vscode/settings.json
   ```

## Run

### Simulation

```
uv run robotpy sim
```

### Deploy to Robot

First, connect to the robot network. Then run:

```
uv run robotpy deploy
```

### Test

```
uv run robotpy test
```

### Type checking

We use mypy to check our type hints in CI. You can run mypy locally:

```sh
uv run mypy .
```

## Code Structure

We use RobotPy's Magicbot framework

`robot.py`: Entry point, has mapping from driver inputs to high level robot actions.

`components/`: Abstracts hardware into robot actions.

`controllers/`: Automates robot actions, mostly with state machines.

`autonomous/`: Controls robot during autonomous period.

`ids.py`: Has CAN ids, PH channels and other port numbers.
