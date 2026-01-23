package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Seconds;
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
// import frc.robot.util.ArmVisualizer3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  // private final ArmVisualizer3d armVisualizer3d;

  public Indexer(IndexerIO io) {
    this.io = io;
    // this.armVisualizer3d = new ArmVisualizer3d("indexer", new Translation3d(0, 0, 0.159),
    // 		Rotation2d.fromDegrees(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // armVisualizer3d.setArmAngle(inputs.position);
    // armVisualizer3d.publish();
  }

  public Voltage getVoltage() {
    return this.inputs.WheelAppliedVoltage;
  }

  public Current getSupplyCurrent() {
    return this.inputs.WheelSupplyCurrent;
  }

  public AngularVelocity getVelocity() {
    return this.inputs.WheelVelocity;
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.stop();
        },
        this);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return Commands.runOnce(() -> io.setVoltage(volts.get()), this);
  }
  ;

  public Command sysIdRoutine() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(3.9),
                null,
                (state) -> Logger.recordOutput("SysId/indexer", state.toString())),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log -> {
                  Logger.recordOutput("SysId/indexer/Voltage", inputs.WheelAppliedVoltage);
                  Logger.recordOutput("SysId/indexer/Position", inputs.WheelPosition);
                  Logger.recordOutput("SysId/indexer/Velocity", inputs.WheelVelocity);
                  log.motor("indexer")
                      .voltage(inputs.WheelAppliedVoltage)
                      .angularPosition(inputs.WheelPosition)
                      .angularVelocity(inputs.WheelVelocity);
                },
                this));

    Command routineCommand =
        new SequentialCommandGroup(
            routine.dynamic(Direction.kForward).withTimeout(Seconds.of(2.75)),
            Commands.waitSeconds(1),
            routine.dynamic(Direction.kReverse).withTimeout(Seconds.of(0.25)),
            Commands.waitSeconds(1),
            routine.quasistatic(Direction.kForward).withTimeout(Seconds.of(4.8)),
            Commands.waitSeconds(1),
            routine.quasistatic(Direction.kReverse).withTimeout(Seconds.of(0.75)));

    return routineCommand;
  }
}
