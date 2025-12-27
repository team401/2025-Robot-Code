package frc.robot.constants;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import frc.robot.constants.AutoStrategyContainer.Action;
import java.util.List;

public class AutoStrategy {

  @JSONExclude public String autoStrategyName = "";
  public List<Action> actions;
}
