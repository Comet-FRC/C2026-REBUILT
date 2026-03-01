package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX wheelLeader = new TalonFX(FlywheelConstants.FLYWHEEL_LEADER_ID);
  private final TalonFX wheelFollower = new TalonFX(FlywheelConstants.FLYWHEEL_FOLLOWER_ID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double desiredVelocityRadPerSec = 0.0;
  private boolean wheelVoltageMode = false;
  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);

  // Cached status signals — refreshed in one batched CAN call
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Temperature> leftDeviceTempSignal;

  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Temperature> rightDeviceTempSignal;

  public FlywheelIOReal() {
    // Cache status signal references
    rightPositionSignal = wheelLeader.getPosition();
    rightVelocitySignal = wheelLeader.getVelocity();
    rightMotorVoltageSignal = wheelLeader.getMotorVoltage();
    rightSupplyCurrentSignal = wheelLeader.getSupplyCurrent();
    rightDeviceTempSignal = wheelLeader.getDeviceTemp();

    leftPositionSignal = wheelFollower.getPosition();
    leftVelocitySignal = wheelFollower.getVelocity();
    leftMotorVoltageSignal = wheelFollower.getMotorVoltage();
    leftSupplyCurrentSignal = wheelFollower.getSupplyCurrent();
    leftDeviceTempSignal = wheelFollower.getDeviceTemp();

    configureMotors();
  }

  private void configureMotors() {
    // Leader configuration
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 20;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 70;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leaderConfig.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;

    // Slot 0 Configs (PID + FF)
    leaderConfig.Slot0.kP = FlywheelConstants.WHEEL_kP;
    leaderConfig.Slot0.kI = FlywheelConstants.WHEEL_kI;
    leaderConfig.Slot0.kD = FlywheelConstants.WHEEL_kD;
    leaderConfig.Slot0.kS = FlywheelConstants.WHEEL_kS;
    leaderConfig.Slot0.kV = FlywheelConstants.WHEEL_kV;
    leaderConfig.Slot0.kA = FlywheelConstants.WHEEL_kA;

    wheelLeader.getConfigurator().apply(leaderConfig);

    // Follower configuration
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.CurrentLimits.StatorCurrentLimit = 80;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelFollower.getConfigurator().apply(followerConfig);

    // Follower opposes leader direction (counter-rotating flywheels)
    wheelFollower.setControl(
        new Follower(FlywheelConstants.FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (wheelVoltageMode) {
      wheelLeader.setControl(voltageRequest.withOutput(wheelDesiredVoltage.in(Volts)));
    } else {
      // Convert rad/s to rot/s for TalonFX
      System.out.println(desiredVelocityRadPerSec);
      double desiredRotationsPerSec = desiredVelocityRadPerSec / (2 * Math.PI);
      wheelLeader.setControl(velocityRequest.withVelocity(desiredRotationsPerSec));
    }

    // Batch-refresh all status signals in one CAN operation
    BaseStatusSignal.refreshAll(
        rightPositionSignal,
        rightVelocitySignal,
        rightMotorVoltageSignal,
        rightSupplyCurrentSignal,
        rightDeviceTempSignal,
        leftPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal,
        leftSupplyCurrentSignal,
        leftDeviceTempSignal);

    // Read from cached signals — no additional CAN traffic
    inputs.rightPosition = Radians.of(rightPositionSignal.getValueAsDouble() * 2 * Math.PI);
    inputs.rightVelocity =
        RadiansPerSecond.of(rightVelocitySignal.getValueAsDouble() * 2 * Math.PI);
    inputs.rightDesiredVelocity = RadiansPerSecond.of(desiredVelocityRadPerSec);
    inputs.rightVelocitySetpoint = RadiansPerSecond.of(desiredVelocityRadPerSec);
    inputs.rightAppliedVolts = Volts.of(rightMotorVoltageSignal.getValueAsDouble());
    inputs.rightSupplyCurrent = Amps.of(rightSupplyCurrentSignal.getValueAsDouble());
    inputs.rightTemperature = Celsius.of(rightDeviceTempSignal.getValueAsDouble());

    inputs.leftPosition = Radians.of(leftPositionSignal.getValueAsDouble() * 2 * Math.PI);
    inputs.leftVelocity = RadiansPerSecond.of(leftVelocitySignal.getValueAsDouble() * 2 * Math.PI);
    inputs.leftDesiredVelocity = RadiansPerSecond.of(desiredVelocityRadPerSec);
    inputs.leftVelocitySetpoint = RadiansPerSecond.of(desiredVelocityRadPerSec);
    inputs.leftAppliedVolts = Volts.of(leftMotorVoltageSignal.getValueAsDouble());
    inputs.leftSupplyCurrent = Amps.of(leftSupplyCurrentSignal.getValueAsDouble());
    inputs.leftTemperature = Celsius.of(leftDeviceTempSignal.getValueAsDouble());
  }

  @Override
  public void stopWheel() {
    wheelLeader.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public void setWheelVoltage(Voltage voltage) {
    wheelVoltageMode = true;
    wheelDesiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    wheelVoltageMode = false;
    desiredVelocityRadPerSec = velocity.in(RadiansPerSecond);
  }

  @Override
  public void enabledInit() {
    // No explicit reset needed for TalonFX velocity control usually, unless we want to clear
    // integral accumulators
  }
}
