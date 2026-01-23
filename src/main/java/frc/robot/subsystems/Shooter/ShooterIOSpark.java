// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkUtil;

public class ShooterIOSpark implements ShooterIO {

  private final SparkMax wheelMotor =
      new SparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax pivotMotor =
      new SparkMax(ShooterConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

  private final ArmFeedforward pivotFF =
      new ArmFeedforward(
          ShooterConstants.PIVOT_kS,
          ShooterConstants.PIVOT_kG,
          ShooterConstants.PIVOT_kV,
          ShooterConstants.PIVOT_kA);

  private final SimpleMotorFeedforward wheelFF =
      new SimpleMotorFeedforward(
          ShooterConstants.WHEEL_kS, ShooterConstants.WHEEL_kV, ShooterConstants.WHEEL_kA);

  private final ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          ShooterConstants.PIVOT_kP,
          ShooterConstants.PIVOT_kI,
          ShooterConstants.PIVOT_kD,
          new TrapezoidProfile.Constraints(2.5, 5));

  private final ProfiledPIDController wheelPID =
      new ProfiledPIDController(
          ShooterConstants.WHEEL_kP,
          ShooterConstants.WHEEL_kI,
          ShooterConstants.WHEEL_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  public ShooterIOSpark() {
    this.configureWheelMotor();
    this.configurePivotMotor();
    this.pivotPID.reset(ShooterConstants.STARTING_ANGLE.in(Radians));
    this.pivotPID.setGoal(ShooterConstants.STARTING_ANGLE.in(Radians));

    this.wheelPID.reset(0);
  }

  private void configureWheelMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    // config.closedLoop
    // 		.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // 		.p(ShooterConstants.WHEEL_kP)
    // 		.i(ShooterConstants.WHEEL_kI)
    // 		.d(ShooterConstants.WHEEL_kD);

    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configurePivotMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40); // TODO: Check if this is enough current
    config
        .encoder
        .positionConversionFactor(ShooterConstants.PIVOT_CONVERSION_FACTOR)
        .velocityConversionFactor(ShooterConstants.PIVOT_CONVERSION_FACTOR / 60.0);
    // config.closedLoop
    // 		.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // 		.p(ShooterConstants.PIVOT_kP)
    // 		.i(ShooterConstants.PIVOT_kI)
    // 		.d(ShooterConstants.PIVOT_kD)
    // 		.outputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkUtil.tryUntilOk(
        pivotMotor,
        5,
        () -> pivotMotor.getEncoder().setPosition(ShooterConstants.STARTING_ANGLE.in(Radians)));
  }

  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
  private boolean wheelVoltageMode = false;

  // private final MutAngle pivotDesiredPosition = IntakeConstants.STARTING_ANGLE.mutableCopy();
  private final MutVoltage pivotDesiredVoltage = Volts.mutable(0);
  private boolean pivotVoltageMode = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (pivotVoltageMode) {
      this.pivotMotor.setVoltage(pivotDesiredVoltage.copy());
      this.pivotPID.reset(
          pivotMotor.getEncoder().getPosition(), pivotMotor.getEncoder().getVelocity());
    } else {
      double pid = pivotPID.calculate(pivotMotor.getEncoder().getPosition());
      double ff =
          pivotFF.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
      double volts;
      volts = pid + ff;
      volts = MathUtil.clamp(volts, -12, 12);

      // Logger.recordOutput("Intake/pivotClosedLoopPID", pid);
      // Logger.recordOutput("Intake/pivotClosedLoopFF", ff);
      // Logger.recordOutput("Intake/pivotClosedLoopAppliedVolts", volts);
      // Logger.recordOutput("Intake/closedLoopError", pivotPID.getPositionError());
      this.pivotMotor.setVoltage(Volts.of(volts));
    }

    if (wheelVoltageMode) {
      this.wheelMotor.setVoltage(wheelDesiredVoltage.copy());
      this.wheelPID.reset(wheelMotor.getEncoder().getVelocity());
    } else {
      double pid = wheelPID.calculate(wheelMotor.getEncoder().getVelocity());
      // double ff = wheelFF.calculate(wheelPID.getSetpoint().position);
      double volts = pid;
      volts = MathUtil.clamp(volts, -12, 12);
      this.wheelMotor.setVoltage(Volts.of(volts));
    }

    inputs.wheelPosition = Radians.of(wheelMotor.getEncoder().getPosition());
    inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getGoal().position);
    inputs.wheelVelocitySetpoint = RadiansPerSecond.of(wheelPID.getSetpoint().position);
    inputs.wheelAppliedVolts = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
    inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
    inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());

    /*
    inputs.pivotPosition = Radians.of(pivotMotor.getEncoder().getPosition());
    inputs.pivotDesiredPosition = Radians.of(pivotPID.getGoal().position);
    // inputs.pivotDesiredPosition = this.pivotDesiredPosition.copy();
    inputs.pivotDesiredPositionSetpoint = Radians.of(this.pivotPID.getSetpoint().position);
    inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getEncoder().getVelocity());
    inputs.pivotAppliedVolts = Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
    inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getOutputCurrent());
    inputs.pivotTemperature = Celsius.of(pivotMotor.getMotorTemperature()); */
  }

  @Override
  public void stopWheel() {
    wheelMotor.setVoltage(0);
  }

  @Override
  public void setWheelVoltage(Voltage voltage) {
    this.wheelVoltageMode = true;
    this.wheelDesiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    this.wheelVoltageMode = false;
    this.wheelPID.reset(wheelMotor.getEncoder().getVelocity());
    this.wheelPID.setGoal(velocity.in(RadiansPerSecond));
  }

  @Override
  public void enabledInit() {
    this.pivotPID.reset(
        this.pivotMotor.getEncoder().getPosition(), this.pivotMotor.getEncoder().getVelocity());

    this.wheelPID.reset(this.wheelMotor.getEncoder().getVelocity());
  }
}
