package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class IndexerIOSim implements IndexerIO {
  private final FlywheelSim rollerSim =
      new FlywheelSim(
          DCMotor.getNEO(1),
          IndexerConstants.GEAR_RATIO,
          0.004); 

  private final ProfiledPIDController rollerPID =
      new ProfiledPIDController(
          IndexerConstants.ROLLER_SIM_kP,
          IndexerConstants.ROLLER_SIM_kI,
          IndexerConstants.ROLLER_SIM_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutVoltage rollerDesiredVoltage = Volts.mutable(0);
  private boolean rollerVoltageMode = false;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    rollerSim.update(0.02); // Update with 20ms dt

    inputs.rollerVelocity = RadiansPerSecond.of(rollerSim.getAngularVelocityRadPerSec());
    inputs.rollerAppliedVolts = rollerDesiredVoltage.copy();
    inputs.rollerSupplyCurrent =
        Amps.of(rollerSim.getCurrentDrawAmps()); // Returns current in amps

    if (rollerVoltageMode) {
      inputs.rollerVelocitySetpoint = inputs.rollerVelocity;
    }

    inputs.rollerDesiredVelocity = RadiansPerSecond.of(rollerPID.getSetpoint().position);

    Logger.recordOutput("Indexer/SimVelocity", inputs.rollerVelocity.in(RadiansPerSecond));
  }

  @Override
  public void stopRoller() {
    rollerVoltageMode = true;
    rollerDesiredVoltage.mut_replace(0, Volts);
    rollerSim.setInputVoltage(0);
  }

  @Override
  public void setRollerVelocitySetpoint(AngularVelocity velocity) {
    rollerVoltageMode = false;
    rollerPID.setGoal(velocity.in(RadiansPerSecond));

    double pidOutput =
        rollerPID.calculate(rollerSim.getAngularVelocityRadPerSec());
    rollerDesiredVoltage.mut_replace(MathUtil.clamp(pidOutput, -12, 12), Volts);
    rollerSim.setInputVoltage(rollerDesiredVoltage.in(Volts));
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    rollerVoltageMode = true;
    rollerDesiredVoltage.mut_replace(MathUtil.clamp(volts.in(Volts), -12, 12), Volts);
    rollerSim.setInputVoltage(rollerDesiredVoltage.in(Volts));
  }

  @Override
  public void enabledInit() {
    rollerPID.reset(rollerSim.getAngularVelocityRadPerSec());
  }
}
