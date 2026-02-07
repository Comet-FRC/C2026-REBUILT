package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

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

public class Flywheel extends SubsystemBase {
  public final FlywheelIO io;
  public final FlywheelIOInputsAutoLogged inputs;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("flywheel", inputs);
  }

  public Command setWheelVoltage(Supplier<Voltage> voltage) {
    return Commands.run(() -> io.setWheelVoltage(voltage.get()), this);
  }

  public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
    return Commands.runOnce(() -> io.setWheelVelocitySetpoint(velocity.get()), this);
  }

  public Command sysIdRoutineWheel() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(1),
                null,
                (state) -> Logger.recordOutput("SysId/fly-wheel", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setWheelVoltage,
                log -> {
                  Logger.recordOutput("SysId/fly-wheel/Voltage", inputs.wheelAppliedVolts);
                  Logger.recordOutput("SysId/fly-wheel/Velocity", inputs.wheelVelocity);
                  Logger.recordOutput("SysId/fly-wheel/Position", inputs.wheelPosition);
                  log.motor("fly-wheel")
                      .voltage(inputs.wheelAppliedVolts)
                      .angularPosition(inputs.wheelPosition)
                      .angularVelocity(inputs.wheelVelocity);
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
