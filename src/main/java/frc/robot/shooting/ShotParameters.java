package frc.robot.shooting;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Everything the shot calculator figures out, bundled together. The auto-aim command reads these
 * every cycle to control the turret and flywheel.
 */
public record ShotParameters(
    boolean isValid, // Are we actually in range to score?
    Rotation2d turretAngle, // Where the turret should point (robot-relative)
    double flywheelSpeedRPM, // How fast to spin the flywheel
    double hoodAngleDegrees, // What angle the hood should be at
    double distanceToTarget) {} // How far we are from the target (after motion correction)
