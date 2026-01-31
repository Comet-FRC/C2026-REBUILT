package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem that can rotate to face a target heading. The turret automatically compensates
 * for robot rotation to maintain its heading relative to the field.
 */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  /**
   * Get the current turret position relative to the robot.
   *
   * @return The turret position as a Rotation2d
   */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(inputs.position.in(Radians));
  }

  /**
   * Get the target heading for the turret to face the alliance wall. Blue alliance = 0° (turret
   * faces +X direction) Red alliance = 180° (turret faces -X direction)
   *
   * @return The field-relative heading the turret should face
   */
  public Rotation2d getAllianceHeading() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return Rotation2d.fromDegrees(180.0);
    }
    // Default to blue alliance
    return Rotation2d.fromDegrees(0.0);
  }

  /**
   * Calculate the turret position needed to face the alliance wall, compensating for robot heading.
   *
   * @param robotHeading The current robot heading from odometry
   * @return The turret position needed to face the alliance wall
   */
  public Rotation2d calculateTurretSetpoint(Rotation2d robotHeading) {
    // Target field-relative heading
    Rotation2d targetFieldHeading = getAllianceHeading();

    // Turret position = field heading - robot heading + turret offset
    // The turret offset accounts for the turret's mounting orientation
    return targetFieldHeading.minus(robotHeading).plus(TurretConstants.TURRET_STARTING_OFFSET);
  }

  /**
   * Set the turret to a specific position.
   *
   * @param position The target position for the turret
   */
  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  /**
   * Command to set the turret to track the alliance wall. This command continuously updates the
   * turret setpoint to compensate for robot rotation.
   *
   * @param robotHeadingSupplier Supplier for the current robot heading (from Drive subsystem)
   * @return A command that tracks the alliance wall
   */
  public Command trackAllianceWall(Supplier<Rotation2d> robotHeadingSupplier) {
    return Commands.run(
        () -> {
          Rotation2d setpoint = calculateTurretSetpoint(robotHeadingSupplier.get());
          io.setPosition(setpoint);
          Logger.recordOutput("Turret/CalculatedSetpoint", setpoint.getRadians());
        },
        this);
  }

  /**
   * Command to set the turret to a fixed position.
   *
   * @param position The target position
   * @return A command that moves the turret to the position
   */
  public Command goToPosition(Rotation2d position) {
    return Commands.run(() -> io.setPosition(position), this);
  }

  /**
   * Command to apply a voltage to the turret (open-loop).
   *
   * @param volts Supplier for the voltage
   * @return A command that applies the voltage
   */
  public Command setVoltage(Supplier<Voltage> volts) {
    return Commands.run(() -> io.setVoltage(volts.get()), this);
  }

  /** Command to stop the turret. */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  /** Reset the turret encoder to the starting offset position. */
  public void resetToStartingPosition() {
    io.resetPosition(TurretConstants.TURRET_STARTING_OFFSET);
  }

  /**
   * SysId routine for turret characterization.
   *
   * @return A command that runs the full SysId routine
   */
  public Command turretSysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("SysId/turret", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log -> {
                  Logger.recordOutput("SysId/turret/Voltage", inputs.appliedVolts);
                  Logger.recordOutput("SysId/turret/Position", inputs.position);
                  Logger.recordOutput("SysId/turret/Velocity", inputs.velocity);
                  log.motor("turret")
                      .voltage(inputs.appliedVolts)
                      .angularPosition(inputs.position)
                      .angularVelocity(inputs.velocity);
                },
                this));

    return Commands.sequence(
        routine.quasistatic(Direction.kForward),
        Commands.waitSeconds(1),
        routine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(1),
        routine.dynamic(Direction.kForward),
        Commands.waitSeconds(1),
        routine.dynamic(Direction.kReverse));
  }
}
