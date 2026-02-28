package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  public final TurretIO io;
  public final TurretIOInputsAutoLogged inputs;

  public Turret(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);
    Logger.recordOutput("Turret/PositionDegrees", inputs.turretPosition.in(Degrees));
  }

  /** Command to set turret voltage (open loop control) */
  public Command setVoltage(Supplier<Voltage> voltage) {
    System.out.println("Setting turret voltage to: " + voltage.get().toString());
    return Commands.run(() -> io.setVoltage(voltage.get()), this);
  }

  public Command resetPosition(Supplier<Angle> position) {
    return Commands.runOnce(() -> io.resetPosition(position.get()), this);
  }

  /** Command to set turret position setpoint (closed loop control) */
  public Command setPosition(Supplier<Angle> position) {
    return Commands.run(() -> io.setPositionSetpoint(position.get()), this);
  }

  /** Command to move turret to a specific angle and hold */
  public Command goToAngle(Angle angle) {
    return Commands.runOnce(() -> io.setPositionSetpoint(angle), this);
  }

  /** Command to stop the turret */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  /** Get current turret position */
  public Angle getAngle() {
    return inputs.turretPosition;
  }

  /** Get current turret velocity */
  public AngularVelocity getVelocity() {
    return inputs.turretVelocity;
  }

  /** Check if turret is at setpoint (within tolerance) */
  public boolean atSetpoint(Angle tolerance) {
    double error =
        Math.abs(inputs.turretPosition.in(Radians) - inputs.turretDesiredPosition.in(Radians));
    return error < tolerance.in(Radians);
  }

  /** SysId routine for turret characterization */
  public Command sysIdRoutine() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2),
                null,
                (state) -> Logger.recordOutput("SysId/turret", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log -> {
                  Logger.recordOutput("SysId/turret/Voltage", inputs.turretAppliedVolts);
                  Logger.recordOutput("SysId/turret/Velocity", inputs.turretVelocity);
                  Logger.recordOutput("SysId/turret/Position", inputs.turretPosition);
                  log.motor("turret")
                      .voltage(inputs.turretAppliedVolts)
                      .angularPosition(inputs.turretPosition)
                      .angularVelocity(inputs.turretVelocity);
                },
                this));

    Command routineCommand =
        new SequentialCommandGroup(
            routine.dynamic(Direction.kReverse),
            Commands.waitSeconds(1),
            routine.dynamic(Direction.kForward),
            Commands.waitSeconds(1),
            routine.quasistatic(Direction.kReverse),
            Commands.waitSeconds(1),
            routine.quasistatic(Direction.kForward));
    return routineCommand;
  }

  public void enabledInit() {
    this.io.enabledInit();
  }
}
