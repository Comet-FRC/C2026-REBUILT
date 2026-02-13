package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOReal implements HoodIO {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Profiled PID for smooth hood motion (slower than turret since it's a smaller mechanism)
  private final ProfiledPIDController hoodPID =
      new ProfiledPIDController(
          HoodConstants.HOOD_kP,
          HoodConstants.HOOD_kI,
          HoodConstants.HOOD_kD,
          new TrapezoidProfile.Constraints(
              Math.toRadians(180), // max velocity: 180 deg/s
              Math.toRadians(90))); // max accel: 90 deg/s^2

  private final MutVoltage desiredVoltage = Volts.mutable(0);
  private boolean voltageMode = false;

  public HoodIOReal() {
    configureHoodMotor();
    hoodPID.reset(0);
  }

  private void configureHoodMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor direction (adjust if hood moves wrong way)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits — lower than turret since the hood is a smaller mechanism
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Gear ratio so position/velocity readings are in mechanism units
    config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;

    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (voltageMode) {
      hoodMotor.setControl(voltageRequest.withOutput(desiredVoltage.in(Volts)));
      hoodPID.reset(getHoodPositionRad());
    } else {
      double pid = hoodPID.calculate(getHoodPositionRad());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      hoodMotor.setControl(voltageRequest.withOutput(volts));
    }

    inputs.hoodPosition = Radians.of(getHoodPositionRad());
    inputs.hoodVelocity = RadiansPerSecond.of(getHoodVelocityRadPerSec());
    inputs.hoodDesiredPosition = Radians.of(hoodPID.getGoal().position);
    inputs.hoodPositionSetpoint = Radians.of(hoodPID.getSetpoint().position);
    inputs.hoodAppliedVolts = Volts.of(hoodMotor.getMotorVoltage().getValueAsDouble());
    inputs.hoodSupplyCurrent = Amps.of(hoodMotor.getSupplyCurrent().getValueAsDouble());
    inputs.hoodTemperature = Celsius.of(hoodMotor.getDeviceTemp().getValueAsDouble());
  }

  /** Returns hood position in radians (mechanism rotations × 2π) */
  private double getHoodPositionRad() {
    return hoodMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  /** Returns hood velocity in rad/s (mechanism RPS × 2π) */
  private double getHoodVelocityRadPerSec() {
    return hoodMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void stop() {
    hoodMotor.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    voltageMode = true;
    desiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    voltageMode = false;
    // Clamp to physical limits
    double clampedDeg =
        MathUtil.clamp(
            position.in(Degrees),
            HoodConstants.MIN_ANGLE.in(Degrees),
            HoodConstants.MAX_ANGLE.in(Degrees));
    hoodPID.reset(getHoodPositionRad());
    hoodPID.setGoal(Math.toRadians(clampedDeg));
  }

  @Override
  public void enabledInit() {
    hoodPID.reset(getHoodPositionRad());
  }

  @Override
  public void resetPosition(Angle position) {
    hoodMotor.setPosition(position.in(Radians) / (2 * Math.PI));
  }
}
