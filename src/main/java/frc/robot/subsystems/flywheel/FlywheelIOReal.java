package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
}
