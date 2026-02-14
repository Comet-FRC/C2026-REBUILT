package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX wheelLeader = new TalonFX(FlywheelConstants.FLYWHEEL_LEADER_ID);
  private final TalonFX wheelFollower = new TalonFX(FlywheelConstants.FLYWHEEL_FOLLOWER_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final SimpleMotorFeedforward wheelController = new SimpleMotorFeedforward(
      FlywheelConstants.WHEEL_kS,
      FlywheelConstants.WHEEL_kV,
      FlywheelConstants.WHEEL_kA);
      
  private final ProfiledPIDController wheelPID =
      new ProfiledPIDController(
          FlywheelConstants.WHEEL_kP,
          FlywheelConstants.WHEEL_kI,
          FlywheelConstants.WHEEL_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
  private boolean wheelVoltageMode = false;

  public FlywheelIOReal() {
    configureMotors();
    wheelPID.reset(0);
  }

  private void configureMotors() {
    // Leader configuration
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 80;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // 1:1 belt ratio (24T to 24T), no gear reduction
    leaderConfig.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;

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
      wheelPID.reset(getVelocityRadPerSec());
    } else {
      double pid = wheelPID.calculate(getVelocityRadPerSec());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      wheelLeader.setControl(voltageRequest.withOutput(volts));
    }

    inputs.wheelPosition = Radians.of(getPositionRad());
    inputs.wheelVelocity = RadiansPerSecond.of(getVelocityRadPerSec());
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getGoal().position);
    inputs.wheelVelocitySetpoint = RadiansPerSecond.of(wheelPID.getSetpoint().position);
    inputs.wheelAppliedVolts = Volts.of(wheelLeader.getMotorVoltage().getValueAsDouble());
    inputs.wheelSupplyCurrent = Amps.of(wheelLeader.getSupplyCurrent().getValueAsDouble());
    inputs.wheelTemperature = Celsius.of(wheelLeader.getDeviceTemp().getValueAsDouble());
  }

  /** Returns flywheel position in radians (mechanism rotations × 2π) */
  private double getPositionRad() {
    return wheelLeader.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  /** Returns flywheel velocity in rad/s (mechanism RPS × 2π) */
  private double getVelocityRadPerSec() {
    return wheelLeader.getVelocity().getValueAsDouble() * 2 * Math.PI;
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
    wheelPID.reset(getVelocityRadPerSec());
    wheelPID.setGoal(velocity.in(RadiansPerSecond));
  }

  @Override
  public void enabledInit() {
    wheelPID.reset(getVelocityRadPerSec());
  }
}
