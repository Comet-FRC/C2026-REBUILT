package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.TargetMode;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Handles all the math for figuring out where to aim and how fast to spin the flywheel. Accounts
 * for robot motion so we can shoot while driving ("shoot on move"). Uses lookup tables that
 * interpolate between measured data points for flywheel speed, hood angle, and time of flight.
 */
public class ShotCalculator {

  // ──────────────────────────────────────────────────────────────────────────
  //  FEEDING target treemaps
  // ──────────────────────────────────────────────────────────────────────────
  // TODO: Fill these in with real data from testing!
  // Stand at each distance, tune RPM until you make the shot, and record it here.
  // The map interpolates between entries so you don't need every single distance.
  private static final InterpolatingDoubleTreeMap FEEDING_FLYWHEEL_SPEED =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(3.5, 3000.0), // meters -> RPM
          Map.entry(4.302, 2500.0),
          Map.entry(4.823, 2800.0),
          Map.entry(5.3, 2900.0),
          Map.entry(6.479, 3300.0),
          Map.entry(8.145, 3400.0));

  // TODO: Fill these in with real data from testing!
  // Same idea — at each distance, find the hood angle that scores, and record it.
  private static final InterpolatingDoubleTreeMap FEEDING_HOOD_ANGLE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(3.5, 20.0), // meters -> degrees
          Map.entry(4.302, 20.0),
          Map.entry(4.823, 20.0),
          Map.entry(5.3, 20.0),
          Map.entry(6.479, 20.0),
          Map.entry(8.145, 20.0));

  // TODO: Fill these in with real data! Use slow-mo video to measure.
  // This is how long the ball is in the air at each distance — needed for shoot on move.
  // If you set these all to 0, shoot-on-move correction is effectively disabled.
  private static final InterpolatingDoubleTreeMap FEEDING_TIME_OF_FLIGHT =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(3.5, 0.8), // meters -> seconds
          Map.entry(4.283, 0.86),
          Map.entry(4.302, 0.74),
          Map.entry(5.3, 1.2),
          Map.entry(6.479, 1.0),
          Map.entry(8.145, 1.16));

  // ──────────────────────────────────────────────────────────────────────────
  //  HUB target treemaps (higher target → more RPM & steeper hood angles)
  // ──────────────────────────────────────────────────────────────────────────
  // TODO: Fill these in with real data from hub testing!
  private static final InterpolatingDoubleTreeMap HUB_FLYWHEEL_SPEED =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.5, 2500.0), // meters -> RPM  (placeholder)
          Map.entry(2.1, 2600.0),
          Map.entry(2.5, 2700.0),
          Map.entry(2.9, 3000.0),
          Map.entry(3.3, 2800.0),
          Map.entry(3.6, 2900.0),
          Map.entry(3.8, 3100.0),
          Map.entry(4.1, 3200.0),
          Map.entry(4.4, 3300.0),
          Map.entry(4.6, 3300.0),
          Map.entry(4.8, 3300.0));
  // TODO: Fill these in with real data from hub testing!
  private static final InterpolatingDoubleTreeMap HUB_HOOD_ANGLE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.5, 0.0), // meters -> degrees  (placeholder)
          Map.entry(2.1, 0.0),
          Map.entry(2.5, 0.0),
          Map.entry(2.9, 5.0),
          Map.entry(3.3, 5.0),
          Map.entry(3.6, 10.0),
          Map.entry(4.1, 10.0),
          Map.entry(4.4, 15.0),
          Map.entry(4.6, 16.0),
          Map.entry(4.8, 17.0));

  // TODO: Fill these in with real data from hub testing!
  private static final InterpolatingDoubleTreeMap HUB_TIME_OF_FLIGHT =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.5, 0.720), // meters -> seconds  (placeholder)
          Map.entry(2.1, 0.73),
          Map.entry(2.5, 0.810),
          Map.entry(2.9, 0.920),
          Map.entry(3.3, 0.940),
          Map.entry(3.6, 0.960),
          Map.entry(4.1, 0.985),
          Map.entry(4.4, 1.005),
          Map.entry(4.6, 1.020),
          Map.entry(4.8, 1.035));

  // How many times we re-calculate the lookahead to get a more accurate prediction.
  private static final int LOOKAHEAD_ITERATIONS = 5;

  /**
   * Does all the shot math. Give it the robot's pose and velocity, and it tells you where to aim
   * the turret, how fast to spin the flywheel, and what hood angle to use.
   *
   * @param mode Whether we're aiming at a FEEDING target or the HUB.
   */
  public static ShotParameters calculate(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Angle currentTurretPosition, TargetMode mode) {

    // Select the right treemaps and target based on mode
    InterpolatingDoubleTreeMap flywheelMap;
    InterpolatingDoubleTreeMap hoodMap;
    InterpolatingDoubleTreeMap tofMap;
    Translation2d target;

    switch (mode) {
      case HUB:
        flywheelMap = HUB_FLYWHEEL_SPEED;
        hoodMap = HUB_HOOD_ANGLE;
        tofMap = HUB_TIME_OF_FLIGHT;
        target = FieldConstants.getHubTarget();
        break;
      case FEEDING:
      default:
        flywheelMap = FEEDING_FLYWHEEL_SPEED;
        hoodMap = FEEDING_HOOD_ANGLE;
        tofMap = FEEDING_TIME_OF_FLIGHT;
        target = FieldConstants.getTargetForRobot2d(robotPose);
        break;
    }

    Translation2d robotPosition = robotPose.getTranslation();
    double rawDistance = robotPosition.getDistance(target);

    // -- Shoot on move correction --
    // The idea: if we're moving, by the time the ball gets to the target, we'll have
    // drifted. So we predict where we'll BE when the ball arrives and aim from there.
    // We iterate a few times because the time of flight depends on distance, which
    // changes as we account for our motion — the TOF changes as distance changes,
    // so we keep refining until both values are mutually consistent.
    //
    // futurePosition is declared outside the loop so the final value is available
    // for the angle calculation below without re-computing it a second time.
    Translation2d futurePosition = robotPosition; // start: no motion assumed
    double correctedDistance = rawDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double tof = tofMap.get(correctedDistance);

      // Where will we be after the ball's flight time?
      futurePosition =
          new Translation2d(
              robotPosition.getX() + fieldVelocity.vxMetersPerSecond * tof,
              robotPosition.getY() + fieldVelocity.vyMetersPerSecond * tof);

      // Recompute distance from that future position so the next iteration
      // uses the correct (tighter) TOF estimate.
      correctedDistance = futurePosition.getDistance(target);
    }

    // -- Figure out the turret angle --
    // futurePosition already holds the converged prediction from the loop above.

    // This gives us the field angle, but the turret needs a robot-relative angle
    Translation2d vectorToTarget = target.minus(futurePosition);
    Rotation2d fieldAngleToTarget = vectorToTarget.getAngle();

    // Subtract robot heading to get turret angle relative to robot front
    Rotation2d idealTurretRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    // Find the best physical angle for the turret given its limits and current position
    Angle turretAngle = findBestTurretAngle(idealTurretRelativeAngle, currentTurretPosition);

    // Look up the flywheel and hood settings for this distance
    double flywheelSpeedRPM = flywheelMap.get(correctedDistance);
    double hoodAngleDegrees = hoodMap.get(correctedDistance);

    // Are we close enough (and not too far) to actually make this shot?
    boolean isValid =
        correctedDistance >= FieldConstants.MIN_SHOOTING_DISTANCE
            && correctedDistance <= FieldConstants.MAX_SHOOTING_DISTANCE;

    flywheelSpeedRPM = flywheelSpeedRPM * 1.06;

    // Log shoot-on-move internals (everything else is logged in AutoAimCommand)
    Logger.recordOutput("AutoAim/RawDistance", rawDistance);
    Logger.recordOutput("AutoAim/CorrectedDistance", correctedDistance);
    Logger.recordOutput("AutoAim/FlywheelSpeed", flywheelSpeedRPM);
    Logger.recordOutput(
        "AutoAim/FieldVelocity",
        Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));

    return new ShotParameters(
        isValid, turretAngle, flywheelSpeedRPM, hoodAngleDegrees, correctedDistance);
  }

  /**
   * Helper function to handle the Turret's wrapping logic.
   *
   * <p>Our turret can rotate from 0 to 360 degrees. {@code getDegrees()} returns values in (-180,
   * 180], so negative angles (e.g. -45°) must be shifted to their [0, 360] equivalent (315°) via
   * the +360 candidate.
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
    // getDegrees() is in (-180, 180], so for a [0, 360] range:
    //   targetDeg itself   → valid when positive (0–180)
    //   targetDeg + 360    → maps negative angles to [180, 360)
    //   targetDeg - 360    → always < -180, never valid (kept for symmetry)
    double[] candidates = {targetDeg, targetDeg + 360.0, targetDeg - 360.0};

    double bestAngle = currentDeg; // Default to staying put (safety)
    double minError = Double.MAX_VALUE;
    boolean foundValid = false;

    // Check each candidate
    for (double cand : candidates) {
      // 1. Is this candidate physically possible? (Within 0 to 360 limits)
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
    if (!foundValid) {
      // Just clamp to the nearest limit so we aim as close as possible.
      if (targetDeg > maxDeg) return TurretConstants.MAX_ANGLE;
      if (targetDeg < minDeg) return TurretConstants.MIN_ANGLE;
      return Units.Degrees.of(Math.max(minDeg, Math.min(maxDeg, targetDeg)));
    }

    return Units.Degrees.of(bestAngle);
  }
}
