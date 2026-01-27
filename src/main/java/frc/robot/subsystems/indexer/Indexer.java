package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void enabledInit() {
    io.enabledInit();
  }

  public void setRollerVelocity(AngularVelocity velocity) {
    io.setRollerVelocitySetpoint(velocity);
  }

  public void setRollerVoltage(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public void stop() {
    io.stopRoller();
  }

  @AutoLogOutput(key = "Indexer/RollerVelocity")
  public AngularVelocity getRollerVelocity() {
    return inputs.rollerVelocity;
  }

  @AutoLogOutput(key = "Indexer/RollerVelocitySetpoint")
  public AngularVelocity getRollerVelocitySetpoint() {
    return inputs.rollerVelocitySetpoint;
  }

  @AutoLogOutput(key = "Indexer/RollerAppliedVolts")
  public Voltage getRollerAppliedVolts() {
    return inputs.rollerAppliedVolts;
  }

  @AutoLogOutput(key = "Indexer/RollerSupplyCurrent")
  public double getRollerSupplyCurrent() {
    return inputs.rollerSupplyCurrent.in(Amps);
  }

  @AutoLogOutput(key = "Indexer/RollerTemperature")
  public double getRollerTemperature() {
    return inputs.rollerMotorTemperature.in(Celsius);
  }

  public Command setRollerVelocityCommand(AngularVelocity velocity) {
    return this.run(() -> setRollerVelocity(velocity));
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
