package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KickerIOSim implements KickerIO {
  private final DCMotorSim topMotor = configureMotor();
  private final DCMotorSim bottomMotor = configureMotor();

  public static DCMotorSim configureMotor() {
    DCMotor wheelGearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> wheelPlant =
        LinearSystemId.createDCMotorSystem(
            wheelGearbox,
            KickerConstants.WHEEL_MOI,
            KickerConstants.TOP_WHEEL_CONVERSION_FACTOR
                / (2.0 * Math.PI) // TODO: ask why only top conversion factor
            );
    return new DCMotorSim(wheelPlant, wheelGearbox);
  }

  private final PIDController topPID =
      new PIDController(KickerConstants.SIM_kP, KickerConstants.SIM_kI, KickerConstants.SIM_kD);

  private final PIDController bottomPID =
      new PIDController(KickerConstants.SIM_kP, KickerConstants.SIM_kI, KickerConstants.SIM_kD);

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(
          KickerConstants.SIM_kS, KickerConstants.SIM_kV, KickerConstants.SIM_kA);

  private boolean topVoltageMode;
  private boolean bottomVoltageMode;

  private final MutVoltage topAppliedVoltage = Volts.mutable(0);
  private final MutVoltage bottomAppliedVoltage = Volts.mutable(0);

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    topMotor.update(0.02);
    bottomMotor.update(0.02);

    runLoopControl();

    inputs.topWheelVelocity = topMotor.getAngularVelocity();
    inputs.topWheelDesiredVelocity = RadiansPerSecond.of(topPID.getSetpoint());
    inputs.topWheelPosition = topMotor.getAngularPosition();
    inputs.topWheelAppliedVoltage = Volts.of(topMotor.getInputVoltage());
    inputs.topWheelSupplyCurrent = Amps.of(topMotor.getCurrentDrawAmps());

    inputs.bottomWheelVelocity = bottomMotor.getAngularVelocity();
    inputs.bottomWheelDesiredVelocity = RadiansPerSecond.of(topPID.getSetpoint());
    inputs.bottomWheelPosition = bottomMotor.getAngularPosition();
    inputs.bottomWheelAppliedVoltage = Volts.of(bottomMotor.getInputVoltage());
    inputs.bottomWheelSupplyCurrent = Amps.of(bottomMotor.getCurrentDrawAmps());
  }

  public void runLoopControl() {
    if (!topVoltageMode) {

      topMotor.setInputVoltage(
          topPID.calculate(topMotor.getAngularVelocity().in(RadiansPerSecond))
          // +
          // wheelFF.calculate(topPID.getSetpoint())
          );
    }

    if (!bottomVoltageMode) {
      bottomMotor.setInputVoltage(
          bottomPID.calculate(bottomMotor.getAngularVelocity().in(RadiansPerSecond))
          // +
          // wheelFF.calculate(bottomPID.getSetpoint())
          );
    }
  }

  @Override
  public void setWheelVelocitySetpoint(
      AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    this.topVoltageMode = false;
    this.bottomVoltageMode = false;
    this.topPID.setSetpoint(topVelocity.in(RadiansPerSecond));
    this.bottomPID.setSetpoint(bottomVelocity.in(RadiansPerSecond));
  }

  @Override
  public void setTopVoltage(Voltage volts) {
    this.topVoltageMode = true;
    this.topMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void setBottomVoltage(Voltage volts) {
    this.topVoltageMode = true;
    this.bottomMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stop() {
    this.setTopVoltage(Volts.of(0.0));
    this.setBottomVoltage(Volts.of(0.0));
  }
}
