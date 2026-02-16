package frc.robot.shooting;

import edu.wpi.first.units.measure.Angle;

/**
 * Everything the shot calculator figures out, bundled together. The auto-aim command reads these
 * every cycle to control the turret and flywheel.
 */
public record ShotParameters(
    boolean isValid, // Are we actually in range to score?
    Angle turretAngle, // Where the turret should point (robot-relative, can be > 360)
    double flywheelSpeedRPM, // How fast to spin the flywheel
    double hoodAngleDegrees, // What angle the hood should be at
    double distanceToTarget) {} // How far we are from the target (after motion correction)
