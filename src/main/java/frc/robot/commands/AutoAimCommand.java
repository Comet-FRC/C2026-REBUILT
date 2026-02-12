package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;

/**
 * Runs as the turret's default command — always active during the match. Every cycle it figures out
 * where to aim (with shoot-on-move compensation), points the turret, and spins up the flywheel. The
 * PID loop never stops running.
 */
public class AutoAimCommand extends Command {
  private final Drive drive;
  private final Turret turret;
  private final Flywheel flywheel;

  private ShotParameters latestParameters = null;

  public AutoAimCommand(Drive drive, Turret turret, Flywheel flywheel) {
    this.drive = drive;
    this.turret = turret;
    this.flywheel = flywheel;

    // We control turret and flywheel, but we only read from drive (don't require it)
    addRequirements(turret, flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();

    // Run the shot calculator — this handles all the math
    latestParameters = ShotCalculator.calculate(robotPose, fieldVelocity);

    // Tell the turret where to point
    Angle turretAngle = Radians.of(latestParameters.turretAngle().getRadians());
    turret.io.setPositionSetpoint(turretAngle);

    // Spin up the flywheel to the right speed for this distance
    AngularVelocity flywheelSpeed = RPM.of(latestParameters.flywheelSpeedRPM());
    flywheel.io.setWheelVelocitySetpoint(flywheelSpeed);

    // Log for AdvantageScope debugging
    int quadrant = ShotCalculator.getQuadrant(latestParameters.turretAngle());
    Logger.recordOutput("AutoAim/Quadrant", quadrant);
    Logger.recordOutput("AutoAim/TurretTargetDeg", latestParameters.turretAngle().getDegrees());
    Logger.recordOutput("AutoAim/FlywheelTargetRPM", latestParameters.flywheelSpeedRPM());
    Logger.recordOutput("AutoAim/HoodAngleDeg", latestParameters.hoodAngleDegrees());
    Logger.recordOutput("AutoAim/ShotValid", latestParameters.isValid());
    Logger.recordOutput("AutoAim/Distance", latestParameters.distanceToTarget());
  }

  @Override
  public void end(boolean interrupted) {
    turret.io.stop();
    flywheel.io.stopWheel();
  }

  @Override
  public boolean isFinished() {
    return false; // Default command, runs forever
  }

  /** Grab the latest shot parameters (AutoFireCommand needs these). */
  public ShotParameters getLatestParameters() {
    return latestParameters;
  }
}
