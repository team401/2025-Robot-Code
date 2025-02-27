package frc.robot.subsystems.scoring;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.JsonConstants;

public class ReefAvoidanceHelper {
  /**
   * Check if the claw will collide with a reef level
   *
   * @param elevatorHeight Current elevator height
   * @param elevatorGoalHeight Elevator goal height
   * @return True if it will collide/pass a reef level, false if it won't
   */
  public static boolean willPassReefLevel(Distance elevatorHeight, Distance elevatorGoalHeight) {
    return getCollisionHeight(elevatorHeight, elevatorGoalHeight) != null;
  }

  /**
   * Check if the claw would collide with a reef level and get the height to clamp below/above if it
   * would
   *
   * @param elevatorHeight Current elevator height
   * @param elevatorGoalHeight Elevator goal height
   * @return The height if it will collide, null if it won't collide
   */
  public static Distance getCollisionHeight(Distance elevatorHeight, Distance elevatorGoalHeight) {
    if (elevatorHeight.lt(JsonConstants.elevatorConstants.L2MaxHeightBelow)
        && elevatorGoalHeight.lt(JsonConstants.elevatorConstants.L2MaxHeightBelow)) {
      // If the current height and the goal height are both below L2, we won't collide
      return null;
    }

    if (elevatorHeight.gt(JsonConstants.elevatorConstants.L4MinHeightAbove)
        && elevatorGoalHeight.gt(JsonConstants.elevatorConstants.L4MinHeightAbove)) {
      // If the current height and the goal height are both above L4, we won't collide
      return null;
    }

    if (elevatorHeight.lt(elevatorGoalHeight)) {
      return getCollisionHeightUpward(elevatorHeight, elevatorGoalHeight);
    } else {
      return getCollisionHeightDownward(elevatorHeight, elevatorGoalHeight);
    }
  }

  /**
   * Determine whether the claw will pass the wrist as the elevator moves down to its goal height
   *
   * @param elevatorHeight Current elevator height
   * @param elevatorGoalHeight Elevator goal height
   * @return The height if it will collide, null if it won't collide
   */
  public static Distance getCollisionHeightDownward(
      Distance elevatorHeight, Distance elevatorGoalHeight) {
    if (elevatorHeight.gte(JsonConstants.elevatorConstants.L4MinHeightAbove)
        && elevatorGoalHeight.lte(JsonConstants.elevatorConstants.L4MinHeightAbove)) {
      return JsonConstants.elevatorConstants.L4MinHeightAbove;
    }

    return null;
  }

  /**
   * Determine whether the claw will pass the wrist as the elevator moves up to its goal height and
   * return the height it would collide at
   *
   * @param elevatorHeight Current elevator height
   * @param elevatorGoalHeight Elevator goal height
   * @return The height if it will collide, null if it won't collide
   */
  public static Distance getCollisionHeightUpward(
      Distance elevatorHeight, Distance elevatorGoalHeight) {
    if (elevatorHeight.lte(JsonConstants.elevatorConstants.L4MinHeightAbove)
        && elevatorGoalHeight.gte(JsonConstants.elevatorConstants.L4MinHeightAbove)) {
      return JsonConstants.elevatorConstants.L4MinHeightAbove;
    }

    return null;
  }
}
