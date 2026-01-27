package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTunableNumber;

public class IndexerIOReal implements IndexerIO {
  	private final LoggedTunableNumber FLYWHEEL_ANGULAR_ACCELERATION = new LoggedTunableNumber("Shooter/Flywheel Angular Acceleration", 150);

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

  private final MutAngularVelocity rollerDesiredVelocity = RadiansPerSecond.mutable(0);
  private final MutVoltage rollerDesiredVoltage = Volts.mutable(0);
  private boolean rollerVoltageMode = false;

  public IndexerIOReal() {
    configureRollerMotor();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if(rollerVoltageMode) {
      this.rollerMotor.setVoltage(rollerDesiredVoltage.in(Volts));
    }

    inputs.rollerVelocity = RadiansPerSecond.of(rollerMotor.getEncoder().getVelocity());
    inputs.rollerPosition = Radians.of(rollerMotor.getEncoder().getPosition());
    inputs.rollerAppliedVoltage = Volts.of(rollerMotor.getAppliedOutput() * 12.0);
    inputs.rollerSupplyCurrent = Amps.of(rollerMotor.getOutputCurrent());
    inputs.rollerDesiredVelocity = this.rollerDesiredVelocity.copy();

  }

  	@Override
	public void setRollerVelocitySetpoint(AngularVelocity velocity) {
		this.rollerVoltageMode = false;
		double rollerVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);

		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(true)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(SmartDashboard.getNumber("Shooter/topP", 0))
				.i(SmartDashboard.getNumber("Shooter/topI", 0))
				.d(SmartDashboard.getNumber("Shooter/topD", 0))
			.maxMotion
				.maxAcceleration(FLYWHEEL_ANGULAR_ACCELERATION.get());
		rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		rollerMotor.getClosedLoopController().setSetpoint(rollerVelocityRadiansPerSecond, ControlType.kMAXMotionVelocityControl);
		this.rollerDesiredVelocity.mut_replace(velocity);
	}

	@Override
	public void stopRoller() {
		this.setRollerVelocitySetpoint(RadiansPerSecond.of(0));
	}

	@Override
	public void setRollerVoltage(Voltage volts) {
		this.rollerVoltageMode = true;
		this.rollerDesiredVoltage.mut_replace(volts);
	}

  @Override
  public void enabledInit(){
    rollerPID.reset(rollerMotor.getEncoder().getVelocity());
  }

}
