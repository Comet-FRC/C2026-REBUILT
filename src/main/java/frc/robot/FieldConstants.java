package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Where the targets are on the field. The field is split into 4 equal rectangles, each with a
 * target at its center. All coordinates are in meters, blue alliance origin (bottom-left).
 */
public final class FieldConstants {
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 8.21;

  // AprilTag layout (used for Hub position derivation)
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  /** Which target the autoaim system is pointing at. */
  public enum TargetMode {
    FEEDING,
    HUB
  }

  // Split the field in half both ways
  public static final double X_MIDLINE = FIELD_LENGTH_METERS / 2.0; // 8.27m
  public static final double Y_MIDLINE = FIELD_WIDTH_METERS / 2.0; // 4.105m
  public static final Translation3d Q1_TARGET = // Blue, bottom
      new Translation3d(1.869, 1.956, 0.0);

  public static final Translation3d Q2_TARGET = // Blue, top (mirrored across Y)
      new Translation3d(1.869, FIELD_WIDTH_METERS - 1.956, 0.0);

  public static final Translation3d Q3_TARGET = // Red, bottom (mirrored across X)
      new Translation3d(FIELD_LENGTH_METERS - 1.869, 1.956, 0.0);

  public static final Translation3d Q4_TARGET = // Red, top (mirrored across both)
      new Translation3d(FIELD_LENGTH_METERS - 1.869, FIELD_WIDTH_METERS - 1.956, 0.0);

  private static final Translation3d[] BLUE_TARGETS = {Q1_TARGET, Q2_TARGET};
  private static final Translation3d[] RED_TARGETS = {Q3_TARGET, Q4_TARGET};

  // TODO: Set these based on how far your shooter can realistically score from
  public static final double MIN_SHOOTING_DISTANCE = 1.0;
  public static final double MAX_SHOOTING_DISTANCE = 6.0;

  // ──────────────────────────────────────────────────────────────────────────
  //  Hub — alliance-specific scoring target at a different height
  // ──────────────────────────────────────────────────────────────────────────
  public static class Hub {
    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // ── Blue alliance hub (derived from AprilTag 26) ──
    public static final Translation3d blueTopCenterPoint =
        new Translation3d(
            aprilTagLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation3d blueInnerCenterPoint =
        new Translation3d(
            aprilTagLayout.getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    public static final Translation2d blueNearLeftCorner =
        new Translation2d(blueTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d blueNearRightCorner =
        new Translation2d(blueTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d blueFarLeftCorner =
        new Translation2d(blueTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d blueFarRightCorner =
        new Translation2d(blueTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // ── Red alliance hub (derived from AprilTag 4) ──
    public static final Translation3d redTopCenterPoint =
        new Translation3d(
            aprilTagLayout.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation3d redInnerCenterPoint =
        new Translation3d(
            aprilTagLayout.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0, innerHeight);

    public static final Translation2d redNearLeftCorner =
        new Translation2d(redTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d redNearRightCorner =
        new Translation2d(redTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d redFarLeftCorner =
        new Translation2d(redTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d redFarRightCorner =
        new Translation2d(redTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
  }

  /** Figures out which quadrant (1-4) the robot is in based on its position. */
  public static int getRobotQuadrant(Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    if (x < X_MIDLINE) {
      return y > Y_MIDLINE ? 2 : 1; // Blue side: Q1 bottom, Q2 top
    } else {
      return y > Y_MIDLINE ? 4 : 3; // Red side: Q3 bottom, Q4 top
    }
  }

  /**
   * Gets the feeding target for the robot's alliance, picking whichever of the two alliance targets
   * is closer.
   */
  public static Translation3d getTargetForRobot(Pose2d robotPose) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Translation3d[] targets = isRed ? RED_TARGETS : BLUE_TARGETS;

    // Pick the closer of the two alliance targets
    Translation2d robotPos = robotPose.getTranslation();
    Translation3d closest = targets[0];
    double closestDist =
        robotPos.getDistance(new Translation2d(targets[0].getX(), targets[0].getY()));

    for (int i = 1; i < targets.length; i++) {
      double dist = robotPos.getDistance(new Translation2d(targets[i].getX(), targets[i].getY()));
      if (dist < closestDist) {
        closestDist = dist;
        closest = targets[i];
      }
    }
    return closest;
  }

  /** Same as above but just the 2D ground position (ignores height). */
  public static Translation2d getTargetForRobot2d(Pose2d robotPose) {
    Translation3d target = getTargetForRobot(robotPose);
    return new Translation2d(target.getX(), target.getY());
  }

  /** Returns the 2D center of the alliance's Hub (one per alliance). */
  public static Translation2d getHubTarget() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Translation3d hub = isRed ? Hub.redTopCenterPoint : Hub.blueTopCenterPoint;
    return new Translation2d(hub.getX(), hub.getY());
  }

  /**
   * Returns the correct 2D target based on the active mode. FEEDING → closest quadrant feeding
   * target; HUB → alliance hub center.
   */
  public static Translation2d getTargetForMode(TargetMode mode, Pose2d robotPose) {
    return switch (mode) {
      case HUB -> getHubTarget();
      case FEEDING -> getTargetForRobot2d(robotPose);
    };
  }
}
