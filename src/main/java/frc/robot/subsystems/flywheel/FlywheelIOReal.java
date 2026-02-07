package frc.robot.subsystems.flywheel;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;

public class FlywheelIOReal implements FlywheelIO {
  private final SparkMax wheelLeader =
      new SparkMax(FlywheelConstants.FLYWHEEL_LEADER_ID, MotorType.kBrushless);
  private final SparkMax wheelFollower =
      new SparkMax(FlywheelConstants.FLYWHEEL_FOLLOWER_ID, MotorType.kBrushless);

  private void configureWheelMotor() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20) // Lower current limit for battery efficiency
        .voltageCompensation(11.5); // Consistent behavior as battery drains
    // leaderConfig
    /// .encoder
    // .positionConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR)
    // .velocityConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    wheelLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20) // Lower current limit for battery efficiency
        .voltageCompensation(11.5); // Consistent behavior as battery drains
    // followerConfig
    // .encoder
    // .positionConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR)
    // .velocityConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    followerConfig.follow(wheelLeader);
    wheelFollower.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final ProfiledPIDController wheelPID =
      new ProfiledPIDController(
          IntakeConstants.WHEEL_kP,
          IntakeConstants.WHEEL_kI,
          IntakeConstants.WHEEL_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
  private boolean wheelVoltageMode = false;

  public FlywheelIOReal() {
    configureWheelMotor();
    wheelPID.reset(0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (wheelVoltageMode) {
      wheelLeader.setVoltage(wheelDesiredVoltage.copy());
      wheelPID.reset(wheelLeader.getEncoder().getVelocity());
    } else {
      double pid = wheelPID.calculate(wheelLeader.getEncoder().getVelocity());
      double volts = MathUtil.clamp(pid, -12.0, 12.0);
      wheelLeader.setVoltage(Volts.of(volts));
    }

    inputs.wheelPosition = Radians.of(wheelLeader.getEncoder().getPosition());
    inputs.wheelVelocity = RadiansPerSecond.of(wheelLeader.getEncoder().getVelocity());
    inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getGoal().position);
    inputs.wheelVelocitySetpoint = RadiansPerSecond.of(wheelPID.getSetpoint().position);
    inputs.wheelAppliedVolts =
        Volts.of(wheelLeader.getAppliedOutput() * wheelLeader.getBusVoltage());
    inputs.wheelSupplyCurrent = Amps.of(wheelLeader.getOutputCurrent());
    inputs.wheelTemperature = Celsius.of(wheelLeader.getMotorTemperature());
  }

  @Override
  public void stopWheel() {
    wheelLeader.setVoltage(0);
  }

  @Override
  public void setWheelVoltage(Voltage voltage) {
    wheelVoltageMode = true;
    wheelDesiredVoltage.mut_replace(voltage);
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity velocity) {
    wheelVoltageMode = false;
    wheelPID.reset(wheelLeader.getEncoder().getVelocity());
    wheelPID.setGoal(velocity.in(RadiansPerSecond));
  }

  @Override
  public void enabledInit() {
    wheelPID.reset(wheelLeader.getEncoder().getVelocity());
  }
}
