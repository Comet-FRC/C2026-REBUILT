package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Where the targets are on the field. The field is split into 4 equal rectangles, each with a
 * target at its center. All coordinates are in meters, blue alliance origin (bottom-left).
 */
public final class FieldConstants {
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 8.21;

  // Split the field in half both ways
  public static final double X_MIDLINE = FIELD_LENGTH_METERS / 2.0; // 8.27m
  public static final double Y_MIDLINE = FIELD_WIDTH_METERS / 2.0; // 4.105m

  // TODO: Replace these with actual target positions once we know where they are.
  // Right now they're just the center of each quadrant.
  //
  //  +--------+--------+
  //  |  Q1    |  Q2    |
  //  | (TL)   | (TR)   |
  //  +--------+--------+
  //  |  Q3    |  Q4    |
  //  | (BL)   | (BR)   |
  //  +--------+--------+
  //  ^ blue wall        ^ red wall
  //  (0,0) is bottom-left

  // TODO: Set Z to actual target height for all of these
  public static final Translation3d Q1_TARGET = // Top-Left, center at ~(4.1, 6.2)
      new Translation3d(X_MIDLINE / 2.0, Y_MIDLINE + Y_MIDLINE / 2.0, 0.0);

  public static final Translation3d Q2_TARGET = // Top-Right, center at ~(12.4, 6.2)
      new Translation3d(X_MIDLINE + X_MIDLINE / 2.0, Y_MIDLINE + Y_MIDLINE / 2.0, 0.0);

  public static final Translation3d Q3_TARGET = // Bottom-Left, center at ~(4.1, 2.1)
      new Translation3d(X_MIDLINE / 2.0, Y_MIDLINE / 2.0, 0.0);

  public static final Translation3d Q4_TARGET = // Bottom-Right, center at ~(12.4, 2.1)
      new Translation3d(X_MIDLINE + X_MIDLINE / 2.0, Y_MIDLINE / 2.0, 0.0);

  private static final Translation3d[] BLUE_TARGETS = {Q1_TARGET, Q2_TARGET, Q3_TARGET, Q4_TARGET};

  // TODO: Set these based on how far your shooter can realistically score from
  public static final double MIN_SHOOTING_DISTANCE = 1.0;
  public static final double MAX_SHOOTING_DISTANCE = 6.0;

  /** Figures out which quadrant (1-4) the robot is in based on its position. */
  public static int getRobotQuadrant(Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    if (x < X_MIDLINE) {
      return y > Y_MIDLINE ? 1 : 3;
    } else {
      return y > Y_MIDLINE ? 2 : 4;
    }
  }

  /** Gets the target for whatever quadrant the robot is in. Flips for red alliance. */
  public static Translation3d getTargetForRobot(Pose2d robotPose) {
    int quadrant = getRobotQuadrant(robotPose);
    Translation3d blueTarget = BLUE_TARGETS[quadrant - 1];

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    if (isRed) {
      return new Translation3d(
          FIELD_LENGTH_METERS - blueTarget.getX(), blueTarget.getY(), blueTarget.getZ());
    }
    return blueTarget;
  }

  /** Same as above but just the 2D ground position (ignores height). */
  public static Translation2d getTargetForRobot2d(Pose2d robotPose) {
    Translation3d target = getTargetForRobot(robotPose);
    return new Translation2d(target.getX(), target.getY());
  }
}
