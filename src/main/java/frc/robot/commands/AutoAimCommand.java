package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
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
  private final Hood hood;

  private ShotParameters latestParameters = null;

  public AutoAimCommand(Drive drive, Turret turret, Flywheel flywheel, Hood hood) {
    this.drive = drive;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;

    // We control turret, flywheel, and hood, but we only read from drive (don't require it)
    addRequirements(turret, flywheel, hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();

    // Run the shot calculator — this handles all the math
    latestParameters =
        ShotCalculator.calculate(
            robotPose, fieldVelocity, Rotation2d.fromRadians(turret.getPosition().in(Radians)));

    // Tell the turret where to point
    Angle turretAngle = latestParameters.turretAngle();
    turret.io.setPositionSetpoint(turretAngle);

    // Spin up the flywheel to the right speed for this distance
    AngularVelocity flywheelSpeed = RPM.of(latestParameters.flywheelSpeedRPM());
    flywheel.io.setWheelVelocitySetpoint(flywheelSpeed);

    // Set the hood to the right angle for this distance
    Angle hoodAngle = Degrees.of(latestParameters.hoodAngleDegrees());
    hood.io.setPositionSetpoint(hoodAngle);

    // Log for AdvantageScope debugging
    int quadrant = ShotCalculator.getQuadrant(latestParameters.turretAngle());
    Logger.recordOutput("AutoAim/Quadrant", quadrant);
    Logger.recordOutput("AutoAim/TurretTargetDeg", latestParameters.turretAngle().in(Degrees));
    Logger.recordOutput("AutoAim/FlywheelTargetRPM", latestParameters.flywheelSpeedRPM());
    Logger.recordOutput("AutoAim/HoodAngleDeg", latestParameters.hoodAngleDegrees());
    Logger.recordOutput("AutoAim/ShotValid", latestParameters.isValid());
    Logger.recordOutput("AutoAim/Distance", latestParameters.distanceToTarget());
  }

  @Override
  public void end(boolean interrupted) {
    turret.io.stop();
    flywheel.io.stopWheel();
    hood.io.stop();
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
