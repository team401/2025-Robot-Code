# 2025 Robot Code

Competition code for Team 401's 2025 Robot, Hydrus.

<!-- omit from toc -->
## Table of Contents
- [Project Features](#project-features)
  - [JSON Constants](#json-constants)
  - [Strategy Manager](#strategy-manager)
    - [JSON Autos](#json-autos)
    - [Smart Autonomy \& Autonomy Levels](#smart-autonomy--autonomy-levels)
- [Using the Simulator](#using-the-simulator)
  - [Installing dependencies](#installing-dependencies)
  - [Setting up AdvantageScope custom assets folder](#setting-up-advantagescope-custom-assets-folder)
  - [Launching sim and viewing robot position](#launching-sim-and-viewing-robot-position)

## Project Features

### JSON Constants

All constants are loaded using CopperCore's JSON Sync feature, allowing us to quickly update constants without recompiling code. This can save 30-60 seconds per deploy, which adds up to hours over the course of a season.

We use [JsonConstants.java](https://github.com/team401/2025-Robot-Code/blob/main/src/main/java/frc/robot/constants/JsonConstants.java) to handle loading of constants and aliasing the loaded files to easily usable paths.

We also make use of CopperCore's Environment Handler with [config.json](https://github.com/team401/2025-Robot-Code/blob/main/src/main/deploy/constants/config.json), allowing us to specify different sets of constants for different environments. This enabled identical code to run on multiple different test platforms, allowing us to integrate our drivetrain code and individual subsystems on a test drivebase and HITL testing rig before our competition robot even existed.

### Strategy Manager

Our custom [Strategy Manager](https://github.com/team401/2025-Robot-Code/blob/main/src/main/java/frc/robot/StrategyManager.java) coordinated of our robot's actions using JSONSync and handled operator controls through Network Tables.

In autonomous, it acts as an action loader and queuer, handling the sequencing of our auto routines.

In teleop, it handles interfacing with our custom operator interface, [SnakeScreen](https://github.com/team401/SnakeScreen).

#### JSON Autos

StrategyManager used JSONSync to load & parse autos [such as this one](https://github.com/team401/2025-Robot-Code/blob/main/src/main/deploy/auto/4PieceRight.json) and automatically generate a sequence of "Actions." It would then queue these actions, generating a command from each action as it was reached and handling the scheduling and cancelling of commands. Using JSON to store autos allowed us to modify autos without re-compiling, and enabled us to create autos declaratively rather than creating command sequences in code.

#### Smart Autonomy & Autonomy Levels

StrategyManager was also in charge of managing the current autonomy level of the robot. A challenge we faced last season was being able to fully utilize odometry and vision for localization, while still remaining functional when vision and odometry failed. Our solution to this in the 2025 season was to have multiple levels of autonomy:

During teleop, our default state was `Smart` autonomy. This was also referred to as "mixed auto." The driver had full control over driving the robot, and synchronously controlled actions (e.g. pushing a button to start intaking). The operator configured the current game piece, and the current field target (e.g. setting Coral and L4 to score coral on the reef). When the driver pulled the trigger to score, the robot would automatically pick the nearest reef pole based on global odometry. It would then drive itself to about a meter away from that face of the reef using our [Linear Drive State](https://github.com/team401/2025-Robot-Code/blob/78548d16519e17e819c804a9da722a5ba1254a08/src/main/java/frc/robot/subsystems/drive/states/LinearDriveState.java). After getting close enough to the AprilTag on the correct face of the reef, the drivetrain would transition to [Single Tag Lineup mode](https://github.com/team401/2025-Robot-Code/blob/main/src/main/java/frc/robot/subsystems/drive/states/LinearDriveState.java), using single camera measurements with the individual tag to line up very precisely.

We also had `Manual` (also called "low") autonomy. This mode exists in case vision becomes unreliable or our coprocessor dies/loses connection. In this mode, the driver can pull the score trigger to warm up, then manually line up with the reef pole (or the barge/processor) and push a button to score the game piece.

During auto, our autonomy level was locked to `High`, also called "Full Auto." In this mode, the robot automatically executed actions back to back based on an auto strategy outlined in a JSON file. Scoring behaved identically as in `Smart` autonomy, except that the scoring locations were manually determined by the contents of the auto file rather than picking the closest reef pole. We used a wrapper around PathPlanner's On-the-Fly pathfinding to integrate it as a state in our existing drivetrain state machine. This allowed the robot to generate paths back to the coral stations by itself in auto, removing the need for predefined paths.

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
