package frc.robot.subsystems.turret;

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

public class TurretIOReal implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final ProfiledPIDController turretPID =
      new ProfiledPIDController(
          TurretConstants.TURRET_kP,
          TurretConstants.TURRET_kI,
          TurretConstants.TURRET_kD,
          new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2)); // rad/s, rad/s^2

  private final MutVoltage desiredVoltage = Volts.mutable(0);
  private boolean voltageMode = false;

  public TurretIOReal() {
    configureTurretMotor();
    turretPID.reset(0);
    turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void configureTurretMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor direction (adjust if turret rotates wrong way)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Gear ratio for position/velocity readings
    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (voltageMode) {
      turretMotor.setControl(voltageRequest.withOutput(desiredVoltage.in(Volts)));
      turretPID.reset(getTurretPosition());
    } else {
      double pid = turretPID.calculate(getTurretPosition());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      turretMotor.setControl(voltageRequest.withOutput(volts));
    }

    inputs.turretPosition = Radians.of(getTurretPosition());
    inputs.turretVelocity = RadiansPerSecond.of(getTurretVelocity());
    inputs.turretDesiredPosition = Radians.of(turretPID.getGoal().position);
    inputs.turretPositionSetpoint = Radians.of(turretPID.getSetpoint().position);
    inputs.turretAppliedVolts =
        Volts.of(turretMotor.getMotorVoltage().getValueAsDouble());
    inputs.turretSupplyCurrent =
        Amps.of(turretMotor.getSupplyCurrent().getValueAsDouble());
    inputs.turretTemperature =
        Celsius.of(turretMotor.getDeviceTemp().getValueAsDouble());
  }

  private double getTurretPosition() {
    // Returns position in radians (mechanism rotations * 2π)
    return turretMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  private double getTurretVelocity() {
    // Returns velocity in rad/s (mechanism RPS * 2π)
    return turretMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void stop() {
    turretMotor.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    voltageMode = true;
    desiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    voltageMode = false;
    turretPID.reset(getTurretPosition());
    turretPID.setGoal(position.in(Radians));
  }

  @Override
  public void enabledInit() {
    turretPID.reset(getTurretPosition());
  }

  @Override
  public void resetPosition(Angle position) {
    turretMotor.setPosition(position.in(Radians) / (2 * Math.PI));
  }
}
