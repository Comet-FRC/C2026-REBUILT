// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

public class IndexerIOSpark implements IndexerIO {

  private final SparkMax motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(IndexerConstants.kS, IndexerConstants.kV, IndexerConstants.kA);

  public IndexerIOSpark() {
    this.configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    config
        .encoder
        .positionConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IndexerConstants.kP)
        .i(IndexerConstants.kI)
        .d(IndexerConstants.kD);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkUtil.tryUntilOk(motor, 5, () -> motor.getEncoder().setPosition(0));
  }

  MutAngularVelocity topWheelDesiredVelocity = RadiansPerSecond.mutable(0);
  MutVoltage desiredVoltage = Volts.mutable(0);
  MutVoltage rampingVoltage = Volts.mutable(0);
  boolean voltageMode = true;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (voltageMode) {
      this.motor.setVoltage(desiredVoltage.in(Volts));
    }

    inputs.WheelPosition = Radians.of(motor.getEncoder().getPosition());
    inputs.WheelVelocity = RadiansPerSecond.of(motor.getEncoder().getVelocity());
    inputs.WheelAppliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.WheelSupplyCurrent = Amps.of(motor.getOutputCurrent());
    inputs.WheelTemperature = Celsius.of(motor.getMotorTemperature());
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    this.voltageMode = false;
    double velocityRadiansPerSecond = velocity.in(RadiansPerSecond);
    // double feedForward = ff.calculate(velocityRadiansPerSecond);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Indexer/topP", 0))
        .i(SmartDashboard.getNumber("Indexer/topI", 0))
        .d(SmartDashboard.getNumber("Indexer/topD", 0));
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    motor.getClosedLoopController().setReference(velocityRadiansPerSecond, ControlType.kVelocity);

    // System.out.println("topD: " + topMotor.configAccessor.closedLoop.getD());

    this.topWheelDesiredVelocity.mut_replace(velocity);
  }

  @Override
  public void stop() {
    this.setWheelVelocitySetpoint(RadiansPerSecond.of(0));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    this.desiredVoltage.mut_replace(volts);
  }
}
