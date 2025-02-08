package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import java.util.List;

public class AutoStrategy {
  @JSONExclude public String autoStrategyName = "";
  public final List<DesiredLocation> scoringLocations = null;
  public final List<FieldTarget> scoringLevels = null;
  public final DesiredLocation intakeLocation = null;
}
