package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax wheelMotor =
      new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  private void configureWheelMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20) // Lower current limit for battery efficiency
        .voltageCompensation(11.5); // Consistent behavior as battery drains
    config
        .encoder
        .positionConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final ProfiledPIDController wheelPID =
      new ProfiledPIDController(
          IntakeConstants.WHEEL_kP,
          IntakeConstants.WHEEL_kI,
          IntakeConstants.WHEEL_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
  private boolean wheelVoltageMode = false;

  private final TalonFX pivotLeader = new TalonFX(IntakeConstants.PIVOT_LEADER_ID);
  private final TalonFX pivotFollower = new TalonFX(IntakeConstants.PIVOT_FOLLOWER_ID);

  private void configurePivotMotors() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    cfg.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30) // Lower for battery efficiency
            .withSupplyCurrentLimitEnable(true);
    cfg.Voltage.PeakForwardVoltage = 11.5; // Voltage compensation
    cfg.Voltage.PeakReverseVoltage = -11.5;
    cfg.Slot0.kP = IntakeConstants.PIVOT_kP;
    cfg.Slot0.kI = IntakeConstants.PIVOT_kI;
    cfg.Slot0.kD = IntakeConstants.PIVOT_kD;
    cfg.Slot0.kS = IntakeConstants.PIVOT_kS;
    cfg.Slot0.kG = IntakeConstants.PIVOT_kG;
    cfg.Slot0.kV = IntakeConstants.PIVOT_kV;
    cfg.Slot0.kA = IntakeConstants.PIVOT_kA;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    var MotionMagicConfigs = cfg.MotionMagic;
    MotionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(2 * Math.PI);
    MotionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(Math.PI); // rad/s^2
    MotionMagicConfigs.MotionMagicJerk = Units.radiansToRotations(Math.PI); // rad/s^3

    pivotLeader.getConfigurator().apply(cfg);
    pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    pivotLeader.setPosition(Units.radiansToRotations(IntakeConstants.STOW_ANGLE.in(Radians)));
  }

  private double pivotDesiredPositionRad = IntakeConstants.STOW_ANGLE.in(Radians);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public IntakeIOReal() {
    configureWheelMotor();
    configurePivotMotors();
    pivotLeader.setControl(
        motionMagicRequest.withPosition(IntakeConstants.STOW_ANGLE.in(Radians) / (2.0 * Math.PI)));
    wheelPID.reset(0);
  }

  private final MutVoltage pivotDesiredVoltage = Volts.mutable(0);
  private boolean pivotVoltageMode = false;

  // update inputs
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (pivotVoltageMode) {
      pivotLeader.setControl(voltageRequest.withOutput(pivotDesiredVoltage.in(Volts)));
    } else {
      pivotLeader.setControl(
          motionMagicRequest.withPosition(Units.radiansToRotations(pivotDesiredPositionRad)));
    }

    if (wheelVoltageMode) {
      wheelMotor.setVoltage(wheelDesiredVoltage.copy());
      wheelPID.reset(wheelMotor.getEncoder().getVelocity());
    } else {
      double pid = wheelPID.calculate(wheelMotor.getEncoder().getVelocity());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      wheelMotor.setVoltage(Volts.of(volts));
    }

    inputs.wheelPosition = Radians.of(wheelMotor.getEncoder().getPosition());
    inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getGoal().position);
    inputs.wheelVelocitySetpoint = RadiansPerSecond.of(wheelPID.getSetpoint().position);
    inputs.wheelAppliedVolts = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
    inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
    inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());

    inputs.pivotPosition =
        Radians.of(Units.rotationsToRadians(pivotLeader.getPosition().getValueAsDouble()));
    inputs.pivotDesiredPosition = Radians.of(pivotDesiredPositionRad);
    inputs.pivotVelocity =
        RadiansPerSecond.of(Units.rotationsToRadians(pivotLeader.getVelocity().getValueAsDouble()));
    inputs.pivotAppliedVolts = pivotLeader.getMotorVoltage().getValue();
    inputs.pivotSupplyCurrent = pivotLeader.getSupplyCurrent().getValue();
    inputs.pivotTemperature = pivotLeader.getDeviceTemp().getValue();
  }

  // methods to control wheel motor
  @Override
  public void stopWheel() {
    wheelMotor.setVoltage(0);
  }

  @Override
  public void setWheelVoltage(Voltage voltage) {
    wheelVoltageMode = true;
    wheelDesiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    wheelVoltageMode = false;
    wheelPID.reset(wheelMotor.getEncoder().getVelocity());
    wheelPID.setGoal(velocity.in(RadiansPerSecond));
  }

  // methods to control pivot motor
  @Override
  public void stopPivot() {
    setPivotVoltage(Volts.of(0));
  }

  @Override
  public void setPivotPositionSetpoint(Angle position) {
    pivotVoltageMode = false;
    pivotDesiredPositionRad = position.in(Radians);
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    pivotVoltageMode = true;
    pivotDesiredVoltage.mut_replace(volts);
  }

  @Override
  public void enabledInit() {
    wheelPID.reset(wheelMotor.getEncoder().getVelocity());

    if (!pivotVoltageMode) {
      pivotLeader.setControl(
          motionMagicRequest.withPosition(Units.radiansToRotations(pivotDesiredPositionRad)));
    }
  }
}
