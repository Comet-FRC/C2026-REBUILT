package frc.robot.subsystems.indexer;

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

public class Indexer extends SubsystemBase {
  public final IndexerIO io;
  public final IndexerIOInputsAutoLogged inputs;

  public Indexer(IndexerIO IndexerIO) {
    this.io = IndexerIO;
    this.inputs = new IndexerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command stop() {
    return Commands.run(() -> io.setRollerVelocitySetpoint(RadiansPerSecond.of(0)), this);
  }

  public Command setRollerVelocity(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> io.setRollerVelocitySetpoint(speed.get()), this);
  }

  public Command setRollerVoltage(Supplier<Voltage> volts) {
    return Commands.runOnce(() -> io.setRollerVoltage(volts.get()), this);
  }

  public Command rollerSysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("SysId/indexer-roller", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setRollerVoltage,
                log -> {
                  Logger.recordOutput("SysId/indexer-roller/Voltage", inputs.rollerAppliedVolts);
                  Logger.recordOutput("SysId/indexer-roller/Velocity", inputs.rollerVelocity);
                  Logger.recordOutput("SysId/indexer-roller/Position", inputs.rollerPosition);
                  log.motor("indexer-roller")
                      .voltage(inputs.rollerAppliedVolts)
                      .angularPosition(inputs.rollerPosition)
                      .angularVelocity(inputs.rollerVelocity);
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
