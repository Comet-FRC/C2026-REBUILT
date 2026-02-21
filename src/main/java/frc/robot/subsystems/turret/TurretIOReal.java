package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private static final LoggedTunableNumber turretkP =
      new LoggedTunableNumber("Turret/kP", TurretConstants.TURRET_kP);
  private static final LoggedTunableNumber turretkD =
      new LoggedTunableNumber("Turret/kD", TurretConstants.TURRET_kD);

  private final MutVoltage desiredVoltage = Volts.mutable(0);
  private boolean voltageMode = false;
  private Angle targetPosition = Radians.of(0);

  public TurretIOReal() {
    configureTurretMotor();
    resetPosition(Degrees.of(180));
  }

  private void configureTurretMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Was counterClockwise_Positive
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    // Software limits (hard stops)
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.MAX_ANGLE.in(Radians) / (2 * Math.PI); // Rotations
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.MIN_ANGLE.in(Radians) / (2 * Math.PI); // Rotations
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.kP = TurretConstants.TURRET_kP;
    config.Slot0.kI = TurretConstants.TURRET_kI;
    config.Slot0.kD = TurretConstants.TURRET_kD;
    config.Slot0.kS = TurretConstants.TURRET_kS;
    config.Slot0.kV = TurretConstants.TURRET_kV;
    config.Slot0.kA = TurretConstants.TURRET_kA;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.TURRET_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.TURRET_MAX_ACCELERATION;

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    if (turretkP.hasChanged(hashCode()) || turretkD.hasChanged(hashCode())) {
      Slot0Configs slot0 = new Slot0Configs();
      turretMotor.getConfigurator().refresh(slot0);
      slot0.kP = turretkP.get();
      slot0.kD = turretkD.get();
      turretMotor.getConfigurator().apply(slot0);
    }
    if (voltageMode) {
      turretMotor.setControl(voltageRequest.withOutput(desiredVoltage.in(Volts)));
    } else {
      turretMotor.setControl(
          motionMagicRequest.withPosition(targetPosition.in(Radians) / (2 * Math.PI)));
    }

    inputs.turretPosition = Radians.of(getTurretPositionRad());
    inputs.turretVelocity = RadiansPerSecond.of(getTurretVelocityRadPerSec());
    inputs.turretDesiredPosition = targetPosition;
    inputs.turretPositionSetpoint =
        Radians.of(turretMotor.getClosedLoopReference().getValueAsDouble() * 2 * Math.PI);
    inputs.turretAppliedVolts = Volts.of(turretMotor.getMotorVoltage().getValueAsDouble());
    inputs.turretSupplyCurrent = Amps.of(turretMotor.getSupplyCurrent().getValueAsDouble());
    inputs.turretTemperature = Celsius.of(turretMotor.getDeviceTemp().getValueAsDouble());
  }

  private double getTurretPositionRad() {
    return turretMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  private double getTurretVelocityRadPerSec() {
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
    // System.out.println("TurretIOReal setVoltage: " + voltage.toString());
    // System.out.println("Check Voltage: " + turretMotor.get());
  }

  @Override
  public void setPositionSetpoint(Angle position) {
    voltageMode = false;
    targetPosition = position;
  }

  @Override
  public void enabledInit() {
    // No need to reset PID, Motion Magic handles starting from current position seamlessly
  }

  @Override
  public void resetPosition(Angle position) {
    turretMotor.setPosition(position.in(Radians) / (2 * Math.PI));
  }
}
