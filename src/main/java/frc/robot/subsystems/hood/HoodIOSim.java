package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.LoggedTunableNumber;

public class HoodIOSim implements HoodIO {
  private final DCMotorSim hoodMotor = configureHoodMotor();

  private static DCMotorSim configureHoodMotor() {
    DCMotor gearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> plant =
        LinearSystemId.createDCMotorSystem(
            gearbox, HoodConstants.HOOD_MOI, HoodConstants.GEAR_RATIO);
    return new DCMotorSim(plant, gearbox);
  }

  private final LoggedTunableNumber hoodKp =
      new LoggedTunableNumber("HoodSim/kP", HoodConstants.HOOD_SIM_kP);
  private final LoggedTunableNumber hoodKi =
      new LoggedTunableNumber("HoodSim/kI", HoodConstants.HOOD_SIM_kI);
  private final LoggedTunableNumber hoodKd =
      new LoggedTunableNumber("HoodSim/kD", HoodConstants.HOOD_SIM_kD);

  private final PIDController hoodPID =
      new PIDController(
          HoodConstants.HOOD_SIM_kP, HoodConstants.HOOD_SIM_kI, HoodConstants.HOOD_SIM_kD);

  private final SimpleMotorFeedforward hoodFF =
      new SimpleMotorFeedforward(
          HoodConstants.HOOD_SIM_kS, HoodConstants.HOOD_SIM_kV, HoodConstants.HOOD_SIM_kA);

  private boolean voltageMode = false;

  public HoodIOSim() {}

  private void updateTunableValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          hoodPID.setPID(values[0], values[1], values[2]);
        },
        hoodKp,
        hoodKi,
        hoodKd);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    updateTunableValues();

    hoodMotor.update(0.02);

    runLoopControl();

    inputs.hoodPosition = hoodMotor.getAngularPosition();
    inputs.hoodVelocity = hoodMotor.getAngularVelocity();
    inputs.hoodDesiredPosition = Radians.of(hoodPID.getSetpoint());
    inputs.hoodPositionSetpoint = Radians.of(hoodPID.getSetpoint());
    inputs.hoodAppliedVolts = Volts.of(hoodMotor.getInputVoltage());
    inputs.hoodSupplyCurrent = Amps.of(hoodMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!voltageMode) {
      double pidOutput = hoodPID.calculate(hoodMotor.getAngularPosition().in(Radians));
      double ffOutput = hoodFF.calculate(hoodPID.getSetpoint());
      hoodMotor.setInputVoltage(pidOutput + ffOutput);
    }
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    this.voltageMode = false;
    hoodPID.setSetpoint(position.in(Radians));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    hoodMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }

  @Override
  public void resetPosition(Angle position) {
    hoodMotor.setAngle(position.in(Radians));
  }
}
