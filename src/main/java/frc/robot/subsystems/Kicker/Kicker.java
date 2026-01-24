// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Voltage getTopVoltage() {
    return this.inputs.topWheelAppliedVoltage;
  }

  public Voltage getBottomVoltage() {
    return this.inputs.bottomWheelAppliedVoltage;
  }

  public Current getTopSupplyCurrent() {
    return this.inputs.topWheelSupplyCurrent;
  }

  public Current getBottomSupplyCurrent() {
    return this.inputs.bottomWheelSupplyCurrent;
  }

  public AngularVelocity getTopVelocity() {
    return this.inputs.topWheelVelocity;
  }

  public AngularVelocity getBottomVelocity() {
    return this.inputs.bottomWheelVelocity;
  }

  public Command setTopVoltage(Supplier<Voltage> volts) {
    return Commands.runOnce(() -> io.setTopVoltage(volts.get()), this);
  }

  public Command setBottomVoltage(Supplier<Voltage> volts) {
    return Commands.runOnce(() -> io.setBottomVoltage(volts.get()), this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.stop();
        },
        this);
  }

  public Command topSysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("SysId/kicker", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setTopVoltage,
                log -> {
                  Logger.recordOutput("SysId/kicker-top/Voltage", inputs.topWheelAppliedVoltage);
                  Logger.recordOutput("SysId/kicker-top/Velocity", inputs.topWheelVelocity);
                  Logger.recordOutput("SysId/kicker-top/Position", inputs.topWheelPosition);
                  log.motor("kicker-top")
                      .voltage(inputs.topWheelAppliedVoltage)
                      .angularPosition(inputs.topWheelPosition)
                      .angularVelocity(inputs.topWheelVelocity);
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

  public Command bottomSysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("SysId/kicker", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setTopVoltage,
                log -> {
                  Logger.recordOutput("SysId/kicker-bottom/Voltage", inputs.topWheelAppliedVoltage);
                  Logger.recordOutput("SysId/kicker-bottom/Velocity", inputs.topWheelVelocity);
                  Logger.recordOutput("SysId/kicker-bottom/Position", inputs.topWheelPosition);
                  log.motor("kicker-bottom")
                      .voltage(inputs.topWheelAppliedVoltage)
                      .angularPosition(inputs.topWheelPosition)
                      .angularVelocity(inputs.topWheelVelocity);
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
