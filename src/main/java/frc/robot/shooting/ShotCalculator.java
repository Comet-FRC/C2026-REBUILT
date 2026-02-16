package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Handles all the math for figuring out where to aim and how fast to spin the flywheel. Accounts
 * for robot motion so we can shoot while driving ("shoot on move"). Uses lookup tables that
 * interpolate between measured data points for flywheel speed, hood angle, and time of flight.
 */
public class ShotCalculator {

  // TODO: Fill these in with real data from testing!
  // Stand at each distance, tune RPM until you make the shot, and record it here.
  // The map interpolates between entries so you don't need every single distance.
  private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 2500.0), // meters -> RPM
          Map.entry(2.0, 2700.0),
          Map.entry(3.0, 3000.0),
          Map.entry(4.0, 3300.0),
          Map.entry(5.0, 3750.0),
          Map.entry(6.0, 4000.0));

  // TODO: Fill these in with real data from testing!
  // Same idea — at each distance, find the hood angle that scores, and record it.
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 15.0), // meters -> degrees
          Map.entry(2.0, 20.0),
          Map.entry(3.0, 27.0),
          Map.entry(4.0, 31.0),
          Map.entry(5.0, 33.0),
          Map.entry(6.0, 35.0));

  // TODO: Fill these in with real data! Use slow-mo video to measure.
  // This is how long the ball is in the air at each distance — needed for shoot on move.
  // If you set these all to 0, shoot-on-move correction is effectively disabled.
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 0.3), // meters -> seconds
          Map.entry(2.0, 0.5),
          Map.entry(3.0, 0.7),
          Map.entry(4.0, 0.9),
          Map.entry(5.0, 1.1),
          Map.entry(6.0, 1.3));

  // How many times we re-calculate the lookahead to get a more accurate prediction.
  // 5 is plenty — it converges fast.
  private static final int LOOKAHEAD_ITERATIONS = 5;

  /**
   * Does all the shot math. Give it the robot's pose and velocity, and it tells you where to aim
   * the turret, how fast to spin the flywheel, and what hood angle to use.
   */
  public static ShotParameters calculate(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Rotation2d currentTurretAngle) {
    // Figure out which quadrant we're in and grab that target
    Translation2d target = FieldConstants.getTargetForRobot2d(robotPose);
    int activeQuadrant = FieldConstants.getRobotQuadrant(robotPose);
    Translation2d robotPosition = robotPose.getTranslation();

    double rawDistance = robotPosition.getDistance(target);

    // -- Shoot on move correction --
    // The idea: if we're moving, by the time the ball gets to the target, we'll have
    // drifted. So we predict where we'll BE when the ball arrives and aim from there.
    // We iterate a few times because the time of flight depends on distance, which
    // changes as we account for our motion.
    Translation2d correctedTarget = target;
    double correctedDistance = rawDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double tof = TIME_OF_FLIGHT_BY_DISTANCE.get(correctedDistance);

      // Where will we be after the ball's flight time?
      double futureX = robotPosition.getX() + fieldVelocity.vxMetersPerSecond * tof;
      double futureY = robotPosition.getY() + fieldVelocity.vyMetersPerSecond * tof;
      Translation2d futurePosition = new Translation2d(futureX, futureY);

      correctedTarget = target;
      correctedDistance = futurePosition.getDistance(correctedTarget);
    }

    // -- Figure out the turret angle --
    // We need to aim from where we'll BE (future position) to the target
    double futureX =
        robotPosition.getX()
            + fieldVelocity.vxMetersPerSecond * TIME_OF_FLIGHT_BY_DISTANCE.get(correctedDistance);
    double futureY =
        robotPosition.getY()
            + fieldVelocity.vyMetersPerSecond * TIME_OF_FLIGHT_BY_DISTANCE.get(correctedDistance);
    Translation2d futurePosition = new Translation2d(futureX, futureY);

    // This gives us the field angle, but the turret needs a robot-relative angle
    Translation2d vectorToTarget = correctedTarget.minus(futurePosition);
    Rotation2d fieldAngleToTarget = vectorToTarget.getAngle();

    // Subtract robot heading to get turret angle relative to robot front
    Rotation2d idealTurretRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    // Find the best physical angle for the turret given its limits and current position
    Angle turretAngle = findBestTurretAngle(idealTurretRelativeAngle, currentTurretAngle);

    // Look up the flywheel and hood settings for this distance
    double flywheelSpeedRPM = FLYWHEEL_SPEED_BY_DISTANCE.get(correctedDistance);
    double hoodAngleDegrees = HOOD_ANGLE_BY_DISTANCE.get(correctedDistance);

    // Are we close enough (and not too far) to actually make this shot?
    boolean isValid =
        correctedDistance >= FieldConstants.MIN_SHOOTING_DISTANCE
            && correctedDistance <= FieldConstants.MAX_SHOOTING_DISTANCE;

    // Log everything so we can debug in AdvantageScope
    Logger.recordOutput("ShotCalculator/RawDistance", rawDistance);
    Logger.recordOutput("ShotCalculator/CorrectedDistance", correctedDistance);
    Logger.recordOutput("ShotCalculator/TurretAngleDeg", turretAngle.in(edu.wpi.first.units.Units.Degrees));
    Logger.recordOutput("ShotCalculator/FlywheelRPM", flywheelSpeedRPM);
    Logger.recordOutput("ShotCalculator/HoodAngleDeg", hoodAngleDegrees);
    Logger.recordOutput("ShotCalculator/IsValid", isValid);
    Logger.recordOutput("ShotCalculator/ActiveQuadrant", activeQuadrant);
    Logger.recordOutput(
        "ShotCalculator/TargetPosition", new double[] {target.getX(), target.getY()});
    Logger.recordOutput(
        "ShotCalculator/FieldVelocity",
        Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));

    return new ShotParameters(
        isValid, turretAngle, flywheelSpeedRPM, hoodAngleDegrees, correctedDistance);
  }

  /**
   * Finds the best physical angle for the turret to move to, considering the one-directional chain
   * (limited but >360 range) and current position to minimize travel.
   */
  private static Angle findBestTurretAngle(
      Rotation2d targetRelativeAngle, Rotation2d currentRelativeAngle) {
    // Physical limits from constants
    double minDeg = frc.robot.subsystems.turret.TurretConstants.MIN_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    double maxDeg = frc.robot.subsystems.turret.TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Degrees);

    // The target angle is somewhere in [-180, 180]. Be careful with normalization.
    // We want to find k such that (target + k*360) is in [min, max].
    // There might be multiple valid k's (e.g., 20 deg and 380 deg).
    // We choose the one closest to currentRelativeAngle to minimize movement.

    double targetDeg = targetRelativeAngle.getDegrees();
    double currentDeg = currentRelativeAngle.getDegrees();

    // Candidates: target, target+360, target-360...
    // Since range is [0, 450], we likely only need to check k=0, k=1.
    // Or maybe k=-1 if target is large and we want small? No, target is usually normalized.
    // Let's just generate a few reasonable candidates.
    double[] candidates = {
      targetDeg, targetDeg + 360.0, targetDeg - 360.0, targetDeg + 720.0
    };

    double bestAngle = currentDeg; // Default to staying put if everything fails (shouldn't happen)
    double minError = Double.MAX_VALUE;
    boolean foundValid = false;

    for (double cand : candidates) {
      if (cand >= minDeg && cand <= maxDeg) {
        double error = Math.abs(cand - currentDeg);
        if (error < minError) {
          minError = error;
          bestAngle = cand;
          foundValid = true;
        }
      }
    }

    // If no valid angle found in range (e.g. target is barely out of reach in a dead zone),
    // clamp to the closest limit.
    if (!foundValid) {
      // Fallback: just return normalized target if something is weird.
      return edu.wpi.first.units.Units.Degrees.of(targetDeg);
    }

    return edu.wpi.first.units.Units.Degrees.of(bestAngle);
  }

  /** Just tells you which quadrant (1-4) an angle is in. Handy for logging. */
  public static int getQuadrant(Angle robotRelativeAngle) {
    double deg = robotRelativeAngle.in(edu.wpi.first.units.Units.Degrees);
    // Normalize to [-180, 180] for quadrant check if needed, but usually quadrant 1 is 0-90
    // If Angle is 370, that's 10 deg, which is Q1.
    // We should normalize simple degree check.
    deg %= 360;
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;

    if (deg >= 0 && deg < 90) return 1;
    if (deg >= 90 && deg <= 180) return 4;
    if (deg < 0 && deg >= -90) return 2;
    return 3;
  }
}
