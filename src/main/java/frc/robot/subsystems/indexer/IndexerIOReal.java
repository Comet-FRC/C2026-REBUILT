package frc.robot.subsystems.indexer;

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

public class IndexerIOReal implements IndexerIO {
  private final SparkMax rollerMotor =
      new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

  private void configureRollerMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(30);
    config
        .encoder
        .positionConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR)
        .velocityConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final ProfiledPIDController rollerPID =
      new ProfiledPIDController(
          IndexerConstants.ROLLER_kP,
          IndexerConstants.ROLLER_kI,
          IndexerConstants.ROLLER_kD,
          new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));

  private final MutVoltage rollerDesiredVoltage = Volts.mutable(0);
  private boolean rollerVoltageMode = false;

  public IndexerIOReal() {
    configureRollerMotor();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.rollerVelocity =
        RadiansPerSecond.of(rollerMotor.getEncoder().getVelocity());
    inputs.rollerAppliedVolts = Volts.of(rollerMotor.getAppliedOutput() * 12);
    inputs.rollerSupplyCurrent = Amps.of(rollerMotor.getOutputCurrent());
    inputs.rollerMotorTemperature = Celsius.of(rollerMotor.getMotorTemperature());

    if (rollerVoltageMode) {
      inputs.rollerVelocitySetpoint = inputs.rollerVelocity;
    }

    inputs.rollerDesiredVelocity = RadiansPerSecond.of(rollerPID.getSetpoint().position);
  }

  @Override
  public void stopRoller() {
    rollerVoltageMode = true;
    rollerDesiredVoltage.mut_replace(0, Volts);
    rollerMotor.setVoltage(0);
  }

  @Override
  public void setRollerVelocitySetpoint(AngularVelocity velocity) {
    rollerVoltageMode = false;
    rollerPID.setGoal(velocity.in(RadiansPerSecond));

    double pidOutput =
        rollerPID.calculate(
            rollerMotor.getEncoder().getVelocity() * IndexerConstants.GEAR_RATIO);
    rollerDesiredVoltage.mut_replace(MathUtil.clamp(pidOutput, -12, 12), Volts);
    rollerMotor.setVoltage(rollerDesiredVoltage.in(Volts));
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    rollerVoltageMode = true;
    rollerDesiredVoltage.mut_replace(MathUtil.clamp(volts.in(Volts), -12, 12), Volts);
    rollerMotor.setVoltage(rollerDesiredVoltage.in(Volts));
  }

  @Override
  public void enabledInit() {
    rollerPID.reset(rollerMotor.getEncoder().getVelocity() * IndexerConstants.GEAR_RATIO);
  }
}
