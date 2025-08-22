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
  - [Driving the robot and scoring](#driving-the-robot-and-scoring)
  - [Intaking](#intaking)
  - [Testing Autos](#testing-autos)

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

**If certain menus aren't visible in your Sim GUI, select `Workspace` > `Reset` in the top menu bar and your layout should be restored. Sometimes windows can be moved off screen and rendered inaccessible.**

- Open the project in WPILib VSCode
- Press Ctrl+Shift+P and type "Simulate Robot Code" to run the sim. When prompted, select "Sim GUI". This will launch Glass, the WPILib simulator application.
- Open AdvantageScope. Open a `3D Field` tab by clicking the `+` in the top right and picking `3D Field`, or pressing `Alt+3`.
- At the top left of the window, type `Odometry/Robot` into the search bar. Click on `NT:/AdvantageKit/RealOutputs/Odometry/Robot` from the list of options. This will your sidebar to the `Robot` field. For help navigating in AdvantageScope, see their [Navigation Docs](https://docs.advantagescope.org/getting-started/navigation).
- Click and drag the `Robot` field from the sidebar to the bottom of the screen under **`Poses`**. Right click on the entry it creates, and select `Robot 2025` to use Hydrus's CAD model.
  - If `Robot 2025` isn't an option, wait a few seconds to make sure all models have loaded, and then make sure you've [set up your custom assets folder](#setting-up-advantagescope-custom-assets-folder) correctly.
- Next, scroll in the sidebar back up to the search bar. This time, search for `componentPositions`. Select `NT:/AdvantageKit/RealOutputs/componentPositions`. Drag `componentPositions` from the sidebar **on top of** `Robot 2025` in the Poses pane at the bottom of the screen. This will tell AdvantageScope to use the list of poses logged under `componentPositions` to position the elements from the robot's 3D model.
- To verify that everything is working, navigate back to Glass, enable `Autonomous` under **`Robot State`**, and then quickly switch back to AdvantageScope. The robot should drive toward the reef and extend its elevator upward. Lineup is unreliable in sim, so it might not succeed in scoring the first time. Restarting sim or disabling and re-enabling auto can let it try again.
- This setup will be preserved by AdvantageScope every time you reopen the app unless you close the tab.

### Driving the robot and scoring

- Launch the simulator and open AdvantageScope with a 3D sim window as described in [the previous section](#launching-sim-and-viewing-robot-position).
- Find the `System Joysticks` and `Joysticks` windows in the Sim GUI. Drag `Keyboard 0` from `System Joysticks` to `Joystick[0]` in `Joysticks` and `Keyboard 1` to `Joystick[1]`. If you have access to real joysticks, you can use these instead.
- If you enable teleop, you should be able to drive around with W, A, S, and D for translation and J and L for rotation.
- Once you can drive, you can set up keybinds for the intake and score triggers. Intake and score are mapped to the left joystick trigger and right joystick trigger, respectively. According to the [WPILib docs](https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html#joystick-class), the triggers are Button 1.
- To check your keybindings, select `DS` in the top menu and then click on `Joystick 0` and `Joystick 1` to display their menus. Make sure button 1 is bound to something for each joystick (it will probably be `z` for joystick 0 and `m` for joystick 1).
- Now, enable teleop again. Press and hold Button 1 for Joystick 1 (probably `m`) and you should see the robot drive to the nearest reef pole and score. Note that lineup is poorly simulated and it may jitter a lot or miss and not score.

### Intaking

To easily send and read values from Network Tables, we use Elastic Dashboard. You can launch it just like AdvantageScope, from WPILib VSCode. You can also use AdvantageScope, if you click on the slider icon right next to the search bar, although this UI doesn't let you pin values or create a layout.

Our sim uses a network tables subscriber attached to `NT:/SmartDashboard/clawSim/coralAvailable` to determine whether or not the robot can currently intake coral. This is a sub-optimal solution, but it was written before the drivetrain sim was usable and it has survived until now.

- To manage intaking in sim, open Elastic while the sim is running.
- Right click anywhere in the grid and select `+ Add widget`.
- Search for `hasCoral` and drag it into the grid. It should display as a red box. This will turn green when the logged value is `true`. Note that this value is only accurate when the robot is enabled.
- Next, search for `coralAvailable`. You'll have to expand the dropdowns by clicking on the `▶`. Drag `coralAvailable` onto the grid next to `hasCoral`. Right click the `coralAvailable` widget and select `Show As` > `Toggle Switch`. This will allow you to control when the robot can intake coral.
- Arranging your windows so that you can see Elastic, Sim GUI, and AdvantageScope will make the next steps much easier. You can make your Elastic window quite small, as it only needs to show two values. If you move the widgets to the top left of the grid, they won't be cut off when downsizing the Elastic window.
- Now, if you toggle `coralAvailable` to on/true and switch your focus back to the Sim GUI, you should be able to press Button 1 on Joystick 0 (probably `z`) to run the intake. While holding the button, you should see the elevator dip very low to its intake setpoint, and after 1-2 seconds `hasCoral` should become true.

### Testing Autos

Simulation of autos isn't perfect. Our robot is much more reliable in real life than in sim, as many of our gains have been tuned to be accurate to real life and we haven't gone back to re-tune the physics of the sim to reflect these changes. However, by running autos multiple times, sim is still very useful for testing and debugging logic in auto without access to a physical robot.

- Navigate back to Elastic. Add a new widget by searching for "auto chooser". This widget is how we select what auto routine will be run when enabling autonomous. The process for choosing an auto with elastic is identical between the sim and real life.
- Select `4PieceRight` from the auto chooser dropdown. Drive the robot so that it's somewhere in between the barge and the reef, or restart sim to reset its position. Note that if you restart the sim, you will have to re-choose the auto.
- Now, if you disable the robot and enable autonomous from the Sim GUI, you should see the robot drive to the reef and attempt to score. If it drives too far to the sides of the reef with the elevator still up, it's failed to line up due to poor sim physics. Disabling and re-enabling auto should fix it, although you may have to try it multiple times.
- Whenever the robot drives to the intake station, toggle `coralAvailable` to true in Elastic to allow it to intake. The auto should continue as normal after it's done intaking. You can also leave `coralAvailable` on true for the entire auto, which will result in quicker cycle times, at the expense of some realism.

For autos that involve algae (e.g. `1x3Barge1`), you'll need to add `hasAlgae` and `algaeAvailable` to Elastic in addition to the fields for coral. Set `algaeAvailable` to true when the claw would be pressed against an algae on the reef.