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
    Distance[] heights = {
      JsonConstants.elevatorConstants.L4MaxHeightBelow,
      JsonConstants.elevatorConstants.L3MaxHeightBelow,
      JsonConstants.elevatorConstants.L2MaxHeightBelow
    };

    for (Distance height : heights) {
      // If the elevator is above the height and the elevator goal height is below the height, we
      // will collide
      if (elevatorHeight.gte(height) && elevatorGoalHeight.lte(height)) {
        return height;
      }
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
    Distance[] heights = {
      JsonConstants.elevatorConstants.L2MinHeightAbove,
      JsonConstants.elevatorConstants.L3MinHeightAbove,
      JsonConstants.elevatorConstants.L4MinHeightAbove
    };

    for (Distance height : heights) {
      // If the elevator is below the height and the elevator goal height is above the height, we
      // will collide
      if (elevatorHeight.lte(height) && elevatorGoalHeight.gte(height)) {
        return height;
      }
    }

    return null;
  }
}
