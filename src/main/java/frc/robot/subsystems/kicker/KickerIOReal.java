package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerIOReal implements KickerIO {
  private final SparkMax leftMotor =
      new SparkMax(KickerConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(KickerConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  private void configureMotors() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    leftConfig
        .encoder
        .positionConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    rightConfig
        .encoder
        .positionConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final ProfiledPIDController leftPID =
      new ProfiledPIDController(
          KickerConstants.ROLLER_kP,
          KickerConstants.ROLLER_kI,
          KickerConstants.ROLLER_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final ProfiledPIDController rightPID =
      new ProfiledPIDController(
          KickerConstants.ROLLER_kP,
          KickerConstants.ROLLER_kI,
          KickerConstants.ROLLER_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutAngularVelocity desiredVelocity = RadiansPerSecond.mutable(0);
  private final MutVoltage desiredVoltage = Volts.mutable(0);
  private boolean voltageMode = false;

  public KickerIOReal() {
    configureMotors();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    if (voltageMode) {
      leftMotor.setVoltage(desiredVoltage.copy());
      rightMotor.setVoltage(desiredVoltage.copy());
    }

    // Left motor inputs
    inputs.leftVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
    inputs.leftPosition = Radians.of(leftMotor.getEncoder().getPosition());
    inputs.leftAppliedVolts = Volts.of(leftMotor.getAppliedOutput() * 12.0);
    inputs.leftSupplyCurrent = Amps.of(leftMotor.getOutputCurrent());
    inputs.leftDesiredVelocity = this.desiredVelocity.copy();

    // Right motor inputs
    inputs.rightVelocity = RadiansPerSecond.of(rightMotor.getEncoder().getVelocity());
    inputs.rightPosition = Radians.of(rightMotor.getEncoder().getPosition());
    inputs.rightAppliedVolts = Volts.of(rightMotor.getAppliedOutput() * 12.0);
    inputs.rightSupplyCurrent = Amps.of(rightMotor.getOutputCurrent());
    inputs.rightDesiredVelocity = this.desiredVelocity.copy();
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity velocity) {
    this.voltageMode = false;
    double velocityRadiansPerSecond = velocity.in(RadiansPerSecond);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    leftConfig
        .encoder
        .positionConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Kicker/leftP", 0))
        .i(SmartDashboard.getNumber("Kicker/leftI", 0))
        .d(SmartDashboard.getNumber("Kicker/leftD", 0))
        .maxMotion
        .maxAcceleration(150);
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    rightConfig
        .encoder
        .positionConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Kicker/rightP", 0))
        .i(SmartDashboard.getNumber("Kicker/rightI", 0))
        .d(SmartDashboard.getNumber("Kicker/rightD", 0))
        .maxMotion
        .maxAcceleration(150);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    leftMotor
        .getClosedLoopController()
        .setSetpoint(velocityRadiansPerSecond, ControlType.kMAXMotionVelocityControl);
    rightMotor
        .getClosedLoopController()
        .setSetpoint(velocityRadiansPerSecond, ControlType.kMAXMotionVelocityControl);
    this.desiredVelocity.mut_replace(velocity);
  }

  @Override
  public void stop() {
    this.setVelocitySetpoint(RadiansPerSecond.of(0));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    this.desiredVoltage.mut_replace(volts);
  }

  @Override
  public void enabledInit() {
    leftPID.reset(leftMotor.getEncoder().getVelocity());
    rightPID.reset(rightMotor.getEncoder().getVelocity());
  }
}
