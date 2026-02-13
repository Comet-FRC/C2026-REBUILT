package frc.robot.subsystems.hood;

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

public class Hood extends SubsystemBase {
  public final HoodIO io;
  public final HoodIOInputsAutoLogged inputs;

  public Hood(HoodIO io) {
    this.io = io;
    this.inputs = new HoodIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("hood", inputs);
  }

  /** Command to set hood voltage (open loop control) */
  public Command setVoltage(Supplier<Voltage> voltage) {
    return Commands.run(() -> io.setVoltage(voltage.get()), this);
  }

  /** Command to set hood position setpoint (closed loop control) */
  public Command setPosition(Supplier<Angle> position) {
    return Commands.run(() -> io.setPositionSetpoint(position.get()), this);
  }

  /** Command to move hood to a specific angle and hold */
  public Command goToAngle(Angle angle) {
    return Commands.runOnce(() -> io.setPositionSetpoint(angle), this);
  }

  /** Command to stop the hood */
  public Command stop() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  /** Get current hood position */
  public Angle getPosition() {
    return inputs.hoodPosition;
  }

  /** Get current hood velocity */
  public AngularVelocity getVelocity() {
    return inputs.hoodVelocity;
  }

  /** Check if hood is at setpoint (within tolerance) */
  public boolean atSetpoint(Angle tolerance) {
    double error =
        Math.abs(inputs.hoodPosition.in(Radians) - inputs.hoodDesiredPosition.in(Radians));
    return error < tolerance.in(Radians);
  }

  /** SysId routine for hood characterization */
  public Command sysIdRoutine() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second), // Slower ramp — hood is small
                Volts.of(2),
                null,
                (state) -> Logger.recordOutput("SysId/hood", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log -> {
                  Logger.recordOutput("SysId/hood/Voltage", inputs.hoodAppliedVolts);
                  Logger.recordOutput("SysId/hood/Velocity", inputs.hoodVelocity);
                  Logger.recordOutput("SysId/hood/Position", inputs.hoodPosition);
                  log.motor("hood")
                      .voltage(inputs.hoodAppliedVolts)
                      .angularPosition(inputs.hoodPosition)
                      .angularVelocity(inputs.hoodVelocity);
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
