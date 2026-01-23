package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.util.ArmVisualizer3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  // private final ArmVisualizer3d armVisualizer;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
    // this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0.378-0.044,0.184),
    // Rotation2d.fromDegrees(0));
    // this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0.333375,0.196815),
    // Rotation2d.fromDegrees(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // armVisualizer.setArmAngle(inputs.pivotPosition);
    // armVisualizer.publish();
  }

  public Command setWheelVoltage(Supplier<Voltage> voltage) {
    return Commands.runOnce(() -> io.setWheelVoltage(voltage.get()), this);
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
                (state) -> Logger.recordOutput("SysId/shooter-wheel", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setWheelVoltage,
                log -> {
                  Logger.recordOutput("SysId/shooter-wheel/Voltage", inputs.wheelAppliedVolts);
                  Logger.recordOutput("SysId/shooter-wheel/Velocity", inputs.wheelVelocity);
                  Logger.recordOutput("SysId/shooter-wheel/Position", inputs.wheelPosition);
                  log.motor("shooter-wheel")
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
