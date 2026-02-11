package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim wheelMotor = configureWheelMotor();
  private final SingleJointedArmSim pivotMotor = configurePivotMotor();

  private static DCMotorSim configureWheelMotor() {
    DCMotor wheelGearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> wheelPlant =
        LinearSystemId.createDCMotorSystem(wheelGearbox, IntakeConstants.WHEEL_MOI, (18.0 / 24.0));
    return new DCMotorSim(wheelPlant, wheelGearbox);
  }

  private static SingleJointedArmSim configurePivotMotor() {
    return new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(2),
        IntakeConstants.GEAR_RATIO,
        SingleJointedArmSim.estimateMOI(
            IntakeConstants.LENGTH.in(Meters), IntakeConstants.MASS.in(Kilograms)),
        IntakeConstants.LENGTH.in(Meters),
        -0.5,
        IntakeConstants.STOW_ANGLE.in(Radians) + 0.5,
        true,
        IntakeConstants.STOW_ANGLE.in(Radians),
        IntakeConstants.PIVOT_ENCODER_DISTANCE_PER_PULSE,
        0);
  }

  // Tunable PID values for wheel
  private final LoggedTunableNumber wheelKp =
      new LoggedTunableNumber("IntakeSim/Wheel/kP", IntakeConstants.WHEEL_SIM_kP);
  private final LoggedTunableNumber wheelKi =
      new LoggedTunableNumber("IntakeSim/Wheel/kI", IntakeConstants.WHEEL_SIM_kI);
  private final LoggedTunableNumber wheelKd =
      new LoggedTunableNumber("IntakeSim/Wheel/kD", IntakeConstants.WHEEL_SIM_kD);

  // Tunable PID values for pivot
  private final LoggedTunableNumber pivotKp =
      new LoggedTunableNumber("IntakeSim/Pivot/kP", IntakeConstants.PIVOT_SIM_kP);
  private final LoggedTunableNumber pivotKi =
      new LoggedTunableNumber("IntakeSim/Pivot/kI", IntakeConstants.PIVOT_SIM_kI);
  private final LoggedTunableNumber pivotKd =
      new LoggedTunableNumber("IntakeSim/Pivot/kD", IntakeConstants.PIVOT_SIM_kD);

  // Tunable FF values for pivot
  private final LoggedTunableNumber pivotKs =
      new LoggedTunableNumber("IntakeSim/Pivot/kS", IntakeConstants.PIVOT_SIM_kS);
  private final LoggedTunableNumber pivotKg =
      new LoggedTunableNumber("IntakeSim/Pivot/kG", IntakeConstants.PIVOT_SIM_kG);
  private final LoggedTunableNumber pivotKv =
      new LoggedTunableNumber("IntakeSim/Pivot/kV", IntakeConstants.PIVOT_SIM_kV);
  private final LoggedTunableNumber pivotKa =
      new LoggedTunableNumber("IntakeSim/Pivot/kA", IntakeConstants.PIVOT_SIM_kA);

  private final PIDController wheelPID =
      new PIDController(
          IntakeConstants.WHEEL_SIM_kP, IntakeConstants.WHEEL_SIM_kI, IntakeConstants.WHEEL_SIM_kD);

  private final SimpleMotorFeedforward wheelFF =
      new SimpleMotorFeedforward(
          IntakeConstants.WHEEL_SIM_kS, IntakeConstants.WHEEL_SIM_kV, IntakeConstants.WHEEL_SIM_kA);

  private final PIDController pivotPID =
      new PIDController(
          IntakeConstants.PIVOT_SIM_kP, IntakeConstants.PIVOT_SIM_kI, IntakeConstants.PIVOT_SIM_kD);

  private ArmFeedforward pivotFF =
      new ArmFeedforward(
          IntakeConstants.PIVOT_SIM_kS,
          IntakeConstants.PIVOT_SIM_kG,
          IntakeConstants.PIVOT_SIM_kV,
          IntakeConstants.PIVOT_SIM_kA);

  /** true = controlled by voltage, false = controled by PID + FF */
  private boolean pivotVoltageMode = false;

  private Voltage pivotAppliedVoltage = Volts.of(0.0);
  /** true = controlled by voltage, false = controled by PID + FF */
  private boolean wheelVoltageMode = false;

  public IntakeIOSim() {
    this.setPivotPositionSetpoint(IntakeConstants.STOW_ANGLE);
  }

  private void updateTunableValues() {
    // Update wheel PID if changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          wheelPID.setPID(values[0], values[1], values[2]);
        },
        wheelKp,
        wheelKi,
        wheelKd);

    // Update pivot PID if changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          pivotPID.setPID(values[0], values[1], values[2]);
        },
        pivotKp,
        pivotKi,
        pivotKd);

    // Update pivot FF if changed (must recreate since ArmFeedforward is immutable)
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          pivotFF = new ArmFeedforward(values[0], values[1], values[2], values[3]);
        },
        pivotKs,
        pivotKg,
        pivotKv,
        pivotKa);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    updateTunableValues();

    wheelMotor.update(0.02);
    pivotMotor.update(0.02);

    runLoopControl();

    inputs.wheelPosition = wheelMotor.getAngularPosition();
    inputs.wheelVelocity = wheelMotor.getAngularVelocity();
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getSetpoint());
    inputs.wheelAppliedVolts = Volts.of(wheelMotor.getInputVoltage());
    inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getCurrentDrawAmps());

    inputs.pivotPosition = Radians.of(pivotMotor.getAngleRads());
    inputs.pivotDesiredPosition = Radians.of(pivotPID.getSetpoint());
    inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getVelocityRadPerSec());
    inputs.pivotAppliedVolts = this.pivotAppliedVoltage;
    inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!wheelVoltageMode) {
      wheelMotor.setInputVoltage(
          wheelPID.calculate(wheelMotor.getAngularVelocity().in(RadiansPerSecond))
              + wheelFF.calculate(wheelPID.getSetpoint()));
    }

    if (!pivotVoltageMode) {
      pivotMotor.setInputVoltage(
          pivotPID.calculate(pivotMotor.getAngleRads())
              + pivotFF.calculate(pivotPID.getSetpoint(), 0));
    }
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    this.wheelVoltageMode = false;
    wheelPID.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setWheelVoltage(Voltage volts) {
    this.wheelVoltageMode = true;
    wheelMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stopWheel() {
    setWheelVoltage(Volts.of(0.0));
  }

  @Override
  public void setPivotPositionSetpoint(Angle position) {
    this.pivotVoltageMode = false;
    pivotPID.setSetpoint(position.in(Radians));
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    this.pivotVoltageMode = true;
    pivotMotor.setInputVoltage(volts.in(Volts));
    this.pivotAppliedVoltage = volts;
  }

  @Override
  public void stopPivot() {
    setPivotVoltage(Volts.of(0.0));
  }
}
