package frc.robot.subsystems.turret;

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

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretMotor = configureTurretMotor();

  private static DCMotorSim configureTurretMotor() {
    DCMotor gearbox = DCMotor.getKrakenX60(1);
    LinearSystem<N2, N1, N2> plant =
        LinearSystemId.createDCMotorSystem(
            gearbox, TurretConstants.TURRET_MOI, TurretConstants.GEAR_RATIO);
    return new DCMotorSim(plant, gearbox);
  }

  private final LoggedTunableNumber turretKp =
      new LoggedTunableNumber("TurretSim/kP", TurretConstants.TURRET_SIM_kP);
  private final LoggedTunableNumber turretKi =
      new LoggedTunableNumber("TurretSim/kI", TurretConstants.TURRET_SIM_kI);
  private final LoggedTunableNumber turretKd =
      new LoggedTunableNumber("TurretSim/kD", TurretConstants.TURRET_SIM_kD);

  private final PIDController turretPID =
      new PIDController(
          TurretConstants.TURRET_SIM_kP,
          TurretConstants.TURRET_SIM_kI,
          TurretConstants.TURRET_SIM_kD);

  private final SimpleMotorFeedforward turretFF =
      new SimpleMotorFeedforward(
          TurretConstants.TURRET_SIM_kS,
          TurretConstants.TURRET_SIM_kV,
          TurretConstants.TURRET_SIM_kA);

  private boolean voltageMode = false;

  public TurretIOSim() {
    turretPID.enableContinuousInput(0, 2 * Math.PI);
    resetPosition(Degrees.of(180));
  }

  private void updateTunableValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          turretPID.setPID(values[0], values[1], values[2]);
        },
        turretKp,
        turretKi,
        turretKd);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    updateTunableValues();
    runLoopControl();

    inputs.turretPosition = turretMotor.getAngularPosition();
    inputs.turretVelocity = turretMotor.getAngularVelocity();
    inputs.turretDesiredPosition = Radians.of(turretPID.getSetpoint());
    inputs.turretAppliedVolts = Volts.of(turretMotor.getInputVoltage());
    inputs.turretSupplyCurrent = Amps.of(turretMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!voltageMode) {
      double pidOutput = turretPID.calculate(turretMotor.getAngularPosition().in(Radians));
      double ffOutput = turretFF.calculate(turretPID.getSetpoint());
      turretMotor.setInputVoltage(pidOutput + ffOutput);
    }
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    this.voltageMode = false;
    turretPID.setSetpoint(position.in(Radians));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    turretMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }

  @Override
  public void resetPosition(Angle position) {
    turretMotor.setAngle(position.in(Radians));
  }
}
