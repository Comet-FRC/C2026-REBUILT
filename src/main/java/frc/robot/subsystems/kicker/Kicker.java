package frc.robot.subsystems.kicker;

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

public class Kicker extends SubsystemBase {
  public final KickerIO io;
  public final KickerIOInputsAutoLogged inputs;

  public Kicker(KickerIO kickerIO) {
    this.io = kickerIO;
    this.inputs = new KickerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command stop() {
    return Commands.run(() -> io.setVelocitySetpoint(RadiansPerSecond.of(0)), this);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> io.setVelocitySetpoint(speed.get()), this);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return Commands.runOnce(() -> io.setVoltage(volts.get()), this);
  }

  public Command sysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("SysId/kicker", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log -> {
                  Logger.recordOutput("SysId/kicker/LeftVoltage", inputs.leftAppliedVolts);
                  Logger.recordOutput("SysId/kicker/LeftVelocity", inputs.leftVelocity);
                  Logger.recordOutput("SysId/kicker/LeftPosition", inputs.leftPosition);
                  Logger.recordOutput("SysId/kicker/RightVoltage", inputs.rightAppliedVolts);
                  Logger.recordOutput("SysId/kicker/RightVelocity", inputs.rightVelocity);
                  Logger.recordOutput("SysId/kicker/RightPosition", inputs.rightPosition);
                  log.motor("kicker-left")
                      .voltage(inputs.leftAppliedVolts)
                      .angularPosition(inputs.leftPosition)
                      .angularVelocity(inputs.leftVelocity);
                  log.motor("kicker-right")
                      .voltage(inputs.rightAppliedVolts)
                      .angularPosition(inputs.rightPosition)
                      .angularVelocity(inputs.rightVelocity);
                },
                this));

    Command routineCommand =
        new SequentialCommandGroup(
            routine.dynamic(Direction.kForward),
            Commands.waitSeconds(3),
            routine.dynamic(Direction.kReverse),
            Commands.waitSeconds(3),
            routine.quasistatic(Direction.kForward),
            Commands.waitSeconds(3),
            routine.quasistatic(Direction.kReverse));
    return routineCommand;
  }
}
