package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretConstants;
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
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Angle currentTurretPosition) { // Changed from Rotation2d to Angle
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
    Angle turretAngle = findBestTurretAngle(idealTurretRelativeAngle, currentTurretPosition);

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
    Logger.recordOutput(
        "ShotCalculator/TurretAngleDeg", turretAngle.in(Units.Degrees));
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
   * Helper function to handle the Turret's wrapping logic.
   *
   * <p>Our turret can rotate from 0 to 450 degrees (for example). <br>
   * If we need to aim at 10 degrees, but we are currently at 380 degrees, it's faster to rotate to
   * 370 degrees (which is the same direction) than to spin all the way back to 10.
   *
   * @param targetRelativeAngle The angle we *want* to aim at (normalized -180 to 180).
   * @param currentTurretPosition Where the turret actually IS right now.
   * @return The best absolute angle to drive the motor to.
   */
  private static Angle findBestTurretAngle(
      Rotation2d targetRelativeAngle, Angle currentTurretPosition) {
    double minDeg = TurretConstants.MIN_ANGLE.in(Units.Degrees);
    double maxDeg = TurretConstants.MAX_ANGLE.in(Units.Degrees);

    double targetDeg = targetRelativeAngle.getDegrees();
    double currentDeg = currentTurretPosition.in(Units.Degrees);

    // Generate all possible ways to aim at this angle.
    // Since 0 degrees is the same as 360 degrees and 720 degrees...
    // We check: The target itself, target + 1 full rotation, target - 1 full rotation.
    double[] candidates = {targetDeg, targetDeg + 360.0, targetDeg - 360.0, targetDeg + 720.0};

    double bestAngle = currentDeg; // Default to staying put (safety)
    double minError = Double.MAX_VALUE;
    boolean foundValid = false;

    // Check each candidate
    for (double cand : candidates) {
      // 1. Is this candidate physically possible? (Within 0 to 450 limits)
      if (cand >= minDeg && cand <= maxDeg) {
        // 2. How far would we have to move to get there?
        double error = Math.abs(cand - currentDeg);

        // 3. Keep the one that requires the LEAST movement.
        if (error < minError) {
          minError = error;
          bestAngle = cand;
          foundValid = true;
        }
      }
    }

    // Edge Case: What if the target is in a "dead zone" we can't reach?
    // e.g., We can only go 0-450, but the target is at -10.
     if (!foundValid) {
      // Just clamp to the nearest limit so we aim as close as possible.
       if (targetDeg > maxDeg) return TurretConstants.MAX_ANGLE;
       if (targetDeg < minDeg) return TurretConstants.MIN_ANGLE;
       return Units.Degrees.of(Math.max(minDeg, Math.min(maxDeg, targetDeg)));
     }
 
     return Units.Degrees.of(bestAngle);
   }
 
  /** Handy helper to check which quadrant (1, 2, 3, 4) the turret is facing. */
   public static int getQuadrant(Angle robotRelativeAngle) {
     double deg = robotRelativeAngle.in(Units.Degrees);
     deg %= 360;
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;

    if (deg >= 0 && deg < 90) return 1;
    if (deg >= 90 && deg <= 180) return 4;
    if (deg < 0 && deg >= -90) return 2;
    return 3;
  }
}
