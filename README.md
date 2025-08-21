# 2025 Robot Code

Competition code for Team 401's 2025 Robot, Hydrus.

<!-- omit from toc -->
## Table of Contents
- [Project Features](#project-features)
  - [JSON Constants](#json-constants)
  - [Strategy Manager/JSON Autos](#strategy-managerjson-autos)
- [Using the Simulator](#using-the-simulator)
  - [Installing dependencies](#installing-dependencies)
  - [Setting up AdvantageScope custom assets folder](#setting-up-advantagescope-custom-assets-folder)
  - [Launching sim and viewing robot position](#launching-sim-and-viewing-robot-position)

## Project Features

### JSON Constants

All constants are loaded using CopperCore's JSON Sync feature, allowing us to quickly update constants without recompiling code. This can save 30-60 seconds per deploy, which adds up to hours over the course of a season.

We use [JsonConstants.java](https://github.com/team401/2025-Robot-Code/blob/main/src/main/java/frc/robot/constants/JsonConstants.java) to handle loading of constants and aliasing the loaded files to easily usable paths.

We also make use of CopperCore's Environment Handler with [config.json](https://github.com/team401/2025-Robot-Code/blob/main/src/main/deploy/constants/config.json), allowing us to specify different sets of constants for different environments. This enabled identical code to run on multiple different test platforms, allowing us to integrate our drivetrain code and individual subsystems on a test drivebase and HITL testing rig before our competition robot even existed.

### Strategy Manager/JSON Autos

Our custom [Strategy Manager](https://github.com/team401/2025-Robot-Code/blob/main/src/main/java/frc/robot/StrategyManager.java) served to coordinate all of our robot's actions via Network Tables and using a custom loader through JSONSync that enabled us to write our autos using JSON. JSONSync parsed autos [such as this one](https://github.com/team401/2025-Robot-Code/blob/main/src/main/deploy/auto/4PieceRight.json) and automatically generated a sequence of "Actions." It would then queue these actions, generating a command from each action as it was reached and handling the scheduling and cancelling of commands. Using JSON to store autos allowed us to modify autos without re-compiling, and enabled us to create autos declaratively rather than creating command sequences in code.

## Using the Simulator

### Installing dependencies

- Install WPILib 2025.3.1 as described [in Zero-to-Robot](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html).
- Clone this repository:
```sh
git clone https://github.com/team401/2025-Robot-Code
```

### Setting up AdvantageScope custom assets folder
- Launch AdvantageScope, either through:
  - WPILib VSCode : Ctrl+Shift+P -> Start Tool -> AdvantageScope
  - Your system app launcher/start menu
- In the top menu bar, navigate to Help > Use Custom Assets Folder.
  - Navigate to the 2025-Robot-Code folder and select `advantagescope_configs`.
  - **Make sure you choose `advantagescope_configs` and not `Robot_401_2025`.** An easy way to do this is, when the file picker window opens to choose your custom assets folder, remain in `2025-Robot-Code`, click on `advantagescope_configs`, and then press `Ok`. If you select the inner folder, the robot model won't be detected by AdvantageScope.

### Launching sim and viewing robot position
- Open the project in WPILib VSCode
- Press Ctrl+Shift+P and type "Simulate Robot Code" to run the sim. When prompted, select "Sim GUI". This will launch Glass, the WPILib simulator application.
- Open AdvantageScope. Open a `3D Field` tab by clicking the `+` in the top right and picking `3D Field`, or pressing `Alt+3`.
- At the top left of the window, type `Odometry/Robot` into the search bar. Click on `NT:/AdvantageKit/RealOutputs/Odometry/Robot` from the list of options. This will your sidebar to the `Robot` field. For help navigating in AdvantageScope, see their [Navigation Docs](https://docs.advantagescope.org/getting-started/navigation).
- Click and drag the `Robot` field from the sidebar to the bottom of the screen under **`Poses`**. Right click on the entry it creates, and select `Robot 2025` to use Hydrus's CAD model.
  - If `Robot 2025` isn't an option, wait a few seconds to make sure all models have loaded, and then make sure you've [set up your custom assets folder](#setting-up-advantagescope-custom-assets-folder) correctly.
- Next, scroll in the sidebar back up to the search bar. This time, search for `componentPositions`. Select `NT:/AdvantageKit/RealOutputs/componentPositions`. Drag `componentPositions` from the sidebar **on top of** `Robot 2025` in the Poses pane at the bottom of the screen. This will tell AdvantageScope to use the list of poses logged under `componentPositions` to position the elements from the robot's 3D model.
- To verify that everything is working, navigate back to Glass, enable `Autonomous` under **`Robot State`**, and then quickly switch back to AdvantageScope. The robot should drive toward the reef and extend its elevator upward. Lineup is unreliable in sim, so it might not succeed in scoring the first time. Restarting sim or disabling and re-enabling auto can let it try again.
