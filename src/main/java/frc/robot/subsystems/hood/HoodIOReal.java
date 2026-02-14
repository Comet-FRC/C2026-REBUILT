package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOReal implements HoodIO {
  private final SparkMax hoodMotor =
      new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

  // Profiled PID for smooth hood motion
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
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false) // Adjust if hood moves wrong way
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30) // NEO current limit
        .voltageCompensation(11.5);
    // Gear ratio applied to encoder so readings are in mechanism rotations/radians
    config
        .encoder
        .positionConversionFactor(
            (1.0 / HoodConstants.GEAR_RATIO) * 2 * Math.PI) // rotations → mechanism radians
        .velocityConversionFactor(
            (1.0 / HoodConstants.GEAR_RATIO) * 2 * Math.PI / 60.0); // RPM → mechanism rad/s
    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (voltageMode) {
      hoodMotor.setVoltage(desiredVoltage.in(Volts));
      hoodPID.reset(getHoodPositionRad());
    } else {
      double pid = hoodPID.calculate(getHoodPositionRad());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      hoodMotor.setVoltage(volts);
    }

    inputs.hoodPosition = Radians.of(getHoodPositionRad());
    inputs.hoodVelocity = RadiansPerSecond.of(getHoodVelocityRadPerSec());
    inputs.hoodDesiredPosition = Radians.of(hoodPID.getGoal().position);
    inputs.hoodPositionSetpoint = Radians.of(hoodPID.getSetpoint().position);
    inputs.hoodAppliedVolts = Volts.of(hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage());
    inputs.hoodSupplyCurrent = Amps.of(hoodMotor.getOutputCurrent());
    inputs.hoodTemperature = Celsius.of(hoodMotor.getMotorTemperature());
  }

  /** Returns hood position in radians (already converted by encoder config) */
  private double getHoodPositionRad() {
    return hoodMotor.getEncoder().getPosition();
  }

  /** Returns hood velocity in rad/s (already converted by encoder config) */
  private double getHoodVelocityRadPerSec() {
    return hoodMotor.getEncoder().getVelocity();
  }

  @Override
  public void stop() {
    hoodMotor.setVoltage(0);
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
    // SparkMax encoder position can be set via the encoder object
    hoodMotor.getEncoder().setPosition(position.in(Radians));
  }
}
