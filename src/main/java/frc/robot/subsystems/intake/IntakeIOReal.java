package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOReal implements IntakeIO {
  // Wheel motors (SparkMax + NEO)
  private final SparkMax wheelLeader =
      new SparkMax(IntakeConstants.INTAKE_LEADER_ID, MotorType.kBrushless);

  // Pivot motors (SparkMax + NEO)
  private final SparkMax pivotRight =
      new SparkMax(IntakeConstants.PIVOT_RIGHT, MotorType.kBrushless);
  private final SparkMax pivotLeft = new SparkMax(IntakeConstants.PIVOT_LEFT, MotorType.kBrushless);

  // REV Through Bore Encoder (Duty Cycle) for pivot position feedback
  private final DutyCycleEncoder throughBoreEncoder =
      new DutyCycleEncoder(IntakeConstants.THROUGH_BORE_ENCODER_DIO);

  private void configureWheelMotor() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(11.5);
    leftConfig
        .encoder
        .positionConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    wheelLeader.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configurePivotMotors() {
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30) // Lower for battery efficiency
        .voltageCompensation(11.5);
    rightConfig
        .encoder
        .positionConversionFactor(IntakeConstants.PIVOT_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.PIVOT_CONVERSION_FACTOR / 60.0);
    pivotRight.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(11.5)
        .inverted(false);
    pivotLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final ProfiledPIDController wheelPID =
      new ProfiledPIDController(
          IntakeConstants.WHEEL_kP,
          IntakeConstants.WHEEL_kI,
          IntakeConstants.WHEEL_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          IntakeConstants.PIVOT_kP,
          IntakeConstants.PIVOT_kI,
          IntakeConstants.PIVOT_kD,
          new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2)); // rad/s, rad/s^2

  // Tunable PID gains for on-the-fly tuning
  private static final LoggedTunableNumber pivotKp =
      new LoggedTunableNumber("Intake/Pivot/kP", IntakeConstants.PIVOT_kP);
  private static final LoggedTunableNumber pivotKd =
      new LoggedTunableNumber("Intake/Pivot/kD", IntakeConstants.PIVOT_kD);

  private final ArmFeedforward pivotRightArmFF =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS,
          IntakeConstants.PIVOT_kG,
          IntakeConstants.PIVOT_kV,
          IntakeConstants.PIVOT_kA);

  private final ArmFeedforward pivotLeftArmFF =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS, 0.5, IntakeConstants.PIVOT_kV, IntakeConstants.PIVOT_kA);

  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
  private boolean wheelVoltageMode = false;

  private final MutVoltage pivotDesiredVoltage = Volts.mutable(0);
  private boolean pivotVoltageMode = true;
  private double pivotDesiredPositionRad = IntakeConstants.STOW_ANGLE.in(Radians);

  public IntakeIOReal() {
    throughBoreEncoder.setInverted(true);
    configureWheelMotor();
    configurePivotMotors();
    wheelPID.reset(0);
    pivotPID.reset(getThroughBorePositionRad());
    pivotPID.setGoal(IntakeConstants.STOW_ANGLE.in(Radians));
  }

  /** Get pivot position in radians from Through Bore Encoder */
  private double getThroughBorePositionRad() {
    double rawRadians = (throughBoreEncoder.get() * 2.0 * Math.PI) + 0.174533;
    return rawRadians;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double currentPivotPositionRad = getThroughBorePositionRad();

    // Update PID gains if tunable values changed
    if (pivotKp.hasChanged(hashCode()) || pivotKd.hasChanged(hashCode())) {
      pivotPID.setP(pivotKp.get());
      pivotPID.setD(pivotKd.get());
    }

    // Pivot control
    if (pivotVoltageMode) {
      pivotRight.setVoltage(pivotDesiredVoltage.in(Volts));
      pivotLeft.setVoltage(pivotDesiredVoltage.in(Volts));
    } else {
      double pidOutput = pivotPID.calculate(currentPivotPositionRad);
      double leftFF =
          pivotLeftArmFF.calculate(
              pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
      double totalVolts = MathUtil.clamp(pidOutput + leftFF, -11.5, 11.5);
      pivotLeft.setVoltage(totalVolts);

      double rightFF =
          pivotRightArmFF.calculate(
              pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
      double rightVolts = MathUtil.clamp(pidOutput + rightFF, -11.5, 11.5);
      pivotRight.setVoltage(rightVolts);
    }

    // Wheel control
    if (wheelVoltageMode) {
      wheelLeader.setVoltage(wheelDesiredVoltage.copy());
      wheelPID.reset(wheelLeader.getEncoder().getVelocity());
    } else {
      double pid = wheelPID.calculate(wheelLeader.getEncoder().getVelocity());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      wheelLeader.setVoltage(Volts.of(volts));
    }

    // Wheel inputs
    inputs.wheelPosition = Radians.of(wheelLeader.getEncoder().getPosition());
    inputs.wheelVelocity = RadiansPerSecond.of(wheelLeader.getEncoder().getVelocity());
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getGoal().position);
    inputs.wheelVelocitySetpoint = RadiansPerSecond.of(wheelPID.getSetpoint().position);
    inputs.wheelAppliedVolts =
        Volts.of(wheelLeader.getAppliedOutput() * wheelLeader.getBusVoltage());
    inputs.wheelSupplyCurrent = Amps.of(wheelLeader.getOutputCurrent());
    inputs.wheelMotorTemperature = Celsius.of(wheelLeader.getMotorTemperature());

    // Pivot inputs (using Through Bore Encoder for position)
    inputs.pivotPosition = Radians.of(currentPivotPositionRad);
    inputs.pivotDesiredPosition = Radians.of(pivotDesiredPositionRad);
    inputs.pivotVelocity = RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
    inputs.pivotRightAppliedVolts =
        Volts.of(pivotRight.getAppliedOutput() * pivotRight.getBusVoltage());
    inputs.pivotLeftAppliedVolts =
        Volts.of(pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage());

    inputs.pivotRightSupplyCurrent = Amps.of(pivotRight.getOutputCurrent());
    inputs.pivotLeftSupplyCurrent = Amps.of(pivotLeft.getOutputCurrent());
    inputs.pivotRightTemperature = Celsius.of(pivotRight.getMotorTemperature());
    inputs.pivotLeftTemperature = Celsius.of(pivotLeft.getMotorTemperature());

    // Through Bore Encoder raw values
    inputs.throughBoreEncoderPositionRaw = throughBoreEncoder.get();
    inputs.throughBoreEncoderConnected = throughBoreEncoder.isConnected();
  }

  @Override
  public void stopWheel() {
    wheelLeader.setVoltage(0);
  }

  @Override
  public void setWheelVoltage(Voltage voltage) {
    wheelVoltageMode = true;
    wheelDesiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    wheelVoltageMode = false;
    wheelPID.reset(wheelLeader.getEncoder().getVelocity());
    wheelPID.setGoal(velocity.in(RadiansPerSecond));
  }

  @Override
  public void stopPivot() {
    setPivotVoltage(Volts.of(0));
  }

  @Override
  public void setPivotPositionSetpoint(Angle position) {
    pivotVoltageMode = false;
    pivotDesiredPositionRad = position.in(Radians);
    pivotPID.setGoal(pivotDesiredPositionRad);
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    pivotVoltageMode = true;
    pivotDesiredVoltage.mut_replace(volts);
  }

  @Override
  public void enabledInit() {
    wheelPID.reset(wheelLeader.getEncoder().getVelocity());
    pivotPID.reset(getThroughBorePositionRad());
  }
}
