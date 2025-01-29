package frc.robot;

import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfiguration;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorMechanism;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public final class InitSubsystems {
  public static ElevatorSubsystem initElevatorSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        return new ElevatorSubsystem(new ElevatorMechanism(new ElevatorIOTalonFX()));
      case SIM:
        return new ElevatorSubsystem(new ElevatorMechanism(new ElevatorIOSim()));
      case REPLAY:
        throw new UnsupportedOperationException("Elevator replay is not yet implemented.");
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems");
    }
  }

  public static Drive initDriveSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().FrontLeft),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().FrontRight),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().BackLeft),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().BackRight));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(DriveConfiguration.getInstance().FrontLeft),
            new ModuleIOSim(DriveConfiguration.getInstance().FrontRight),
            new ModuleIOSim(DriveConfiguration.getInstance().BackLeft),
            new ModuleIOSim(DriveConfiguration.getInstance().BackRight));

      case MAPLESIM:
        // Sim robot, instantiate physics sim IO implementations
        new SwerveDriveSimulation driveSim =
            new SwerveDriveSimulation(
                DriveTrainSimulationConfig.Default());
        // DrivetrainConstants.SimConstants.driveSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        drive =
            new Drive(
                new GyroIOMapleSim(driveSim.getGyroSimulation()),
                new ModuleIOMapleSim(driveSim.getModules()[0], TunerConstants.FrontLeft),
                new ModuleIOMapleSim(driveSim.getModules()[1], TunerConstants.FrontRight),
                new ModuleIOMapleSim(driveSim.getModules()[2], TunerConstants.BackLeft),
                new ModuleIOMapleSim(driveSim.getModules()[3], TunerConstants.BackRight));
        
        break;

      default:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }
}


<<<<<<< HEAD

  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSim = null;
  private Field2d field2d;
  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;


