package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.TargetMode;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Runs as the turret's default command — always active during the match. Every
 * cycle it figures out
 * where to aim (with shoot-on-move compensation), points the turret, and spins
 * up the flywheel. The
 * PID loop never stops running.
 */
public class AutoAimCommand extends Command {
	private final Turret turret;
	private final Supplier<TargetMode> targetModeSupplier;
	private final Supplier<ChassisSpeeds> driveVelocitySupplier;
	private final Supplier<Pose2d> drivePoseSupplier;

	private ShotParameters latestParameters = null;

	/**
	 * 
	 * @param turret 
	 * @param drivePoseSupplier Drive's field pose
	 * @param driveVelocitySupplier Drive's robot-oriented chassis speeds
	 * @param modeSupplier
	 */
	public AutoAimCommand(
		Turret turret,
		Supplier<Pose2d> drivePoseSupplier,
		Supplier<ChassisSpeeds> driveVelocitySupplier,
		Supplier<TargetMode> modeSupplier
	) {
		this.turret = turret;
		this.targetModeSupplier = modeSupplier;
		this.drivePoseSupplier = drivePoseSupplier;
		this.driveVelocitySupplier = driveVelocitySupplier;

		addRequirements(turret);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		// Calculating the turret's pose
		Pose2d drivePose = drivePoseSupplier.get();
		Transform2d turretOffset = new Transform2d(TurretConstants.OFFSET.unaryMinus(), Meters.zero(), Rotation2d.kZero);
		Pose2d turretPose = drivePose.transformBy(turretOffset);

		// Calculating the turret's field-oriented velocity
		ChassisSpeeds robotVelocity = driveVelocitySupplier.get();
		double tangentialVelocity = TurretConstants.OFFSET.in(Meters) * robotVelocity.omegaRadiansPerSecond;
		ChassisSpeeds turretRobotVelocity = new ChassisSpeeds(
				robotVelocity.vxMetersPerSecond,
				robotVelocity.vyMetersPerSecond - tangentialVelocity,
				robotVelocity.omegaRadiansPerSecond + turret.getVelocity().in(RadiansPerSecond));
		ChassisSpeeds turretFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(turretRobotVelocity,
				drivePose.getRotation());

		// Run the shot calculator
		TargetMode mode = targetModeSupplier.get();
		latestParameters = ShotCalculator.calculate(turretPose, turretFieldVelocity, turret.getAngle(), mode);

		// Tell the turret where to point
		Angle turretAngle = latestParameters.turretAngle();
		turret.io.setPositionSetpoint(turretAngle);

		// Log to AdvantageScope
		Logger.recordOutput("AutoAim/TargetMode", mode.name());
		Logger.recordOutput("AutoAim/TurretTargetDeg", turretAngle.in(Degrees));
		Logger.recordOutput("AutoAim/Distance", latestParameters.distanceToTarget());
		Logger.recordOutput("AutoAim/ShotValid", latestParameters.isValid());
		Logger.recordOutput(
				"AutoAim/ActiveTarget",
				new Pose2d(FieldConstants.getTargetForMode(mode, turretPose), turretPose.getRotation()));
	}

	@Override
	public void end(boolean interrupted) {
		turret.io.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	/** Grab the latest shot parameters. */
	public ShotParameters getLatestParameters() {
		return latestParameters;
	}
}
