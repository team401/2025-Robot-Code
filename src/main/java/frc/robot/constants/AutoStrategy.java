package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import frc.robot.constants.autoStrategies.ExampleAutoPath;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.List;

public interface AutoStrategy {
  public enum ActionType {
    Intake,
    Score
  };

  public static final String autoStrategyName = "";
  public static final JSONSync<ExampleAutoPath> synced = null;

  public final List<DesiredLocation> scoringLocations = null;
  public final List<FieldTarget> scoringLevels = null;
  public final DesiredLocation intakeLocation = null;

  public static record Action(
      ActionType type, GamePiece piece, DesiredLocation location, FieldTarget scoringTarget) {}
}
