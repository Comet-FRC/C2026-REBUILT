package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SparkUtil;

public class KickerIOSpark implements KickerIO {

  private final SparkMax topMotor =
      new SparkMax(KickerConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax bottomMotor =
      new SparkMax(KickerConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

  private final SimpleMotorFeedforward topFF =
      new SimpleMotorFeedforward(
          KickerConstants.TOP_kS, KickerConstants.TOP_kV, KickerConstants.TOP_kA);

  private final SimpleMotorFeedforward bottomFF =
      new SimpleMotorFeedforward(
          KickerConstants.BOTTOM_kS, KickerConstants.BOTTOM_kV, KickerConstants.BOTTOM_kA);

  public KickerIOSpark() {
    this.configureTopMotor();
    this.configureBottomMotor();
  }

  private void configureTopMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(KickerConstants.TOP_WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.TOP_WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(KickerConstants.TOP_kP)
        .i(KickerConstants.TOP_kI)
        .d(KickerConstants.TOP_kD)
        .maxMotion
        .maxAcceleration(100)
        .allowedClosedLoopError(3.75);

    topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkUtil.tryUntilOk(topMotor, 5, () -> topMotor.getEncoder().setPosition(0));
  }

  private void configureBottomMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(KickerConstants.BOTTOM_WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.BOTTOM_WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(KickerConstants.BOTTOM_kP)
        .i(KickerConstants.BOTTOM_kI)
        .d(KickerConstants.BOTTOM_kD)
        .maxMotion
        .maxAcceleration(100)
        .allowedClosedLoopError(3.75);

    bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkUtil.tryUntilOk(bottomMotor, 5, () -> bottomMotor.getEncoder().setPosition(0));
  }

  MutAngularVelocity topDesiredVelocity = RadiansPerSecond.mutable(0);
  MutAngularVelocity bottomDesiredVelocity = RadiansPerSecond.mutable(0);

  MutVoltage topDesiredVoltage = Volts.mutable(0);
  boolean topVoltageMode = false;

  MutVoltage bottomDesiredVoltage = Volts.mutable(0);
  boolean bottomVoltageMode = false;

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    if (topVoltageMode) {
      this.topMotor.setVoltage(topDesiredVoltage.in(Volts));
    }

    if (bottomVoltageMode) {
      this.bottomMotor.setVoltage(bottomDesiredVoltage.in(Volts));
    }

    inputs.topWheelVelocity = RadiansPerSecond.of(topMotor.getEncoder().getVelocity());
    inputs.topWheelDesiredVelocity = this.topDesiredVelocity.copy();
    inputs.topWheelPosition = Radians.of(topMotor.getEncoder().getPosition());
    inputs.topWheelAppliedVoltage =
        Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
    inputs.topWheelSupplyCurrent = Amps.of(topMotor.getOutputCurrent());
    inputs.topTemperature = Celsius.of(topMotor.getMotorTemperature());

    inputs.bottomWheelVelocity = RadiansPerSecond.of(bottomMotor.getEncoder().getVelocity());
    inputs.bottomWheelDesiredVelocity = this.bottomDesiredVelocity.copy();
    inputs.bottomWheelPosition = Radians.of(bottomMotor.getEncoder().getPosition());
    inputs.bottomWheelAppliedVoltage =
        Volts.of(bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage());
    inputs.bottomWheelSupplyCurrent = Amps.of(bottomMotor.getOutputCurrent());
    inputs.bottomTemperature = Celsius.of(bottomMotor.getMotorTemperature());
  }

  @Override
  public void setTopVelocitySetpoint(AngularVelocity velocity) {
    this.topVoltageMode = false;
    double topVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);
    // double topFeedForward = topWheelFF.calculate(topVelocityRadiansPerSecond);
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(KickerConstants.TOP_WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.TOP_WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Shooter/topP", 0))
        .i(SmartDashboard.getNumber("Shooter/topI", 0))
        .d(SmartDashboard.getNumber("Shooter/topD", 0));
    topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    topMotor
        .getClosedLoopController()
        .setReference(topVelocityRadiansPerSecond, ControlType.kVelocity);
    // System.out.println("topD: " + topMotor.configAccessor.closedLoop.getD());
    this.topDesiredVelocity.mut_replace(velocity);
  }

  @Override
  public void setBottomVelocitySetpoint(AngularVelocity velocity) {
    this.topVoltageMode = false;
    double bottomVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);
    // double topFeedForward = topWheelFF.calculate(topVelocityRadiansPerSecond);
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(KickerConstants.BOTTOM_WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(KickerConstants.BOTTOM_WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Shooter/topP", 0))
        .i(SmartDashboard.getNumber("Shooter/topI", 0))
        .d(SmartDashboard.getNumber("Shooter/topD", 0));
    bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    topMotor
        .getClosedLoopController()
        .setReference(bottomVelocityRadiansPerSecond, ControlType.kVelocity);
    // System.out.println("topD: " + topMotor.configAccessor.closedLoop.getD());
    this.bottomDesiredVelocity.mut_replace(velocity);
  }

  @Override
  public void setTopVoltage(Voltage volts) {
    this.topVoltageMode = true;
    this.topDesiredVoltage.mut_replace(volts);
  }

  @Override
  public void setBottomVoltage(Voltage volts) {
    this.bottomVoltageMode = true;
    this.bottomDesiredVoltage.mut_replace(volts);
  }

  @Override
  public void stop() {
    this.setWheelVelocitySetpoint(RadiansPerSecond.of(0), RadiansPerSecond.of(0));
  }
}
