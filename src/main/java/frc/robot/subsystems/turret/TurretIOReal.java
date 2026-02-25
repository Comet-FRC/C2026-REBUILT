package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
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

  // Cached status signals — refreshed in one batched CAN call
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> velocitySignal;
  private final StatusSignal<Double> closedLoopRefSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Temperature> deviceTempSignal;

  public TurretIOReal() {
    // Cache status signal references (these don't do CAN reads yet)
    positionSignal = turretMotor.getPosition();
    velocitySignal = turretMotor.getVelocity();
    closedLoopRefSignal = turretMotor.getClosedLoopReference();
    motorVoltageSignal = turretMotor.getMotorVoltage();
    supplyCurrentSignal = turretMotor.getSupplyCurrent();
    deviceTempSignal = turretMotor.getDeviceTemp();

    configureTurretMotor();
    resetPosition(Degrees.of(180));
  }

  private void configureTurretMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Was counterClockwise_Positive
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
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
    // config.Slot0.IntegralZone = TurretConstants.TURRET_I_ZONE;
    config.Slot0.kV = TurretConstants.TURRET_kV;
    config.Slot0.kA = TurretConstants.TURRET_kA;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.TURRET_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.TURRET_MAX_ACCELERATION;

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Update PID gains if they have changed
    if (turretkP.hasChanged(hashCode()) || turretkD.hasChanged(hashCode())) {
      Slot0Configs slot0 = new Slot0Configs();
      turretMotor.getConfigurator().refresh(slot0);
      slot0.kP = turretkP.get();
      slot0.kD = turretkD.get();
      turretMotor.getConfigurator().apply(slot0);
    }
    // Loop control
    if (voltageMode) {
      turretMotor.setControl(voltageRequest.withOutput(desiredVoltage.in(Volts)));
    } else {
      turretMotor.setControl(
          motionMagicRequest.withPosition(targetPosition.in(Radians) / (2 * Math.PI)));
    }

    // Batch-refresh all status signals in one CAN operation
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        closedLoopRefSignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        deviceTempSignal);

    // Read from cached signals — no additional CAN traffic
    inputs.turretPosition = Radians.of(positionSignal.getValueAsDouble() * 2 * Math.PI);
    inputs.turretVelocity = RadiansPerSecond.of(velocitySignal.getValueAsDouble() * 2 * Math.PI);
    inputs.turretDesiredPosition = targetPosition;
    inputs.turretPositionSetpoint =
        Radians.of(closedLoopRefSignal.getValueAsDouble() * 2 * Math.PI);
    inputs.turretAppliedVolts = Volts.of(motorVoltageSignal.getValueAsDouble());
    inputs.turretSupplyCurrent = Amps.of(supplyCurrentSignal.getValueAsDouble());
    inputs.turretTemperature = Celsius.of(deviceTempSignal.getValueAsDouble());
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
