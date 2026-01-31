package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;

  // Control requests
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private Rotation2d targetPosition = new Rotation2d();

  public TurretIOTalonFX() {
    // Use RIO CAN bus (empty string). Change to "canivore" if using CANivore.
    motor = new TalonFX(TurretConstants.TURRET_MOTOR_ID, "");

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Feedback configuration (gear ratio)
    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    // PID gains
    config.Slot0 = TurretConstants.TURRET_GAINS;

    // MotionMagic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = TurretConstants.MOTION_MAGIC_JERK;

    // Soft limits (if configured)
    if (TurretConstants.FORWARD_SOFT_LIMIT != null) {
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.FORWARD_SOFT_LIMIT;
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    }
    if (TurretConstants.REVERSE_SOFT_LIMIT != null) {
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.REVERSE_SOFT_LIMIT;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    }

    // Apply configuration
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Initialize encoder to starting offset (turret faces 180° from intake at match start)
    tryUntilOk(
        5, () -> motor.setPosition(TurretConstants.TURRET_STARTING_OFFSET.getRotations(), 0.25));

    // Get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    appliedVoltsSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionSignal,
        velocitySignal,
        appliedVoltsSignal,
        statorCurrentSignal,
        supplyCurrentSignal);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            appliedVoltsSignal,
            statorCurrentSignal,
            supplyCurrentSignal);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.position = Radians.of(Units.rotationsToRadians(positionSignal.getValueAsDouble()));
    inputs.velocity =
        RadiansPerSecond.of(Units.rotationsToRadians(velocitySignal.getValueAsDouble()));
    inputs.appliedVolts = appliedVoltsSignal.getValue();
    inputs.statorCurrent = statorCurrentSignal.getValue();
    inputs.supplyCurrent = supplyCurrentSignal.getValue();
    inputs.targetPosition = Radians.of(targetPosition.getRadians());
  }

  @Override
  public void setPosition(Rotation2d position) {
    targetPosition = position;
    motor.setControl(motionMagicRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void resetPosition(Rotation2d position) {
    tryUntilOk(5, () -> motor.setPosition(position.getRotations(), 0.25));
  }
}
