package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Bind this to a button. While held, it watches the turret and flywheel — the moment both are
 * locked on and up to speed, it feeds the kicker to fire. Stops feeding as soon as either drops out
 * of tolerance.
 *
 * <p>Shot parameters (flywheel RPM, hood angle, validity) are read directly from {@link
 * AutoAimCommand#getLatestParameters()} so both commands always agree on the same turret pose and
 * shoot-on-move correction. AutoAimCommand must be running concurrently (it holds the turret
 * requirement; this command does not).
 */
public class AutoFireCommand extends Command {
  private final AutoAimCommand autoAim;
  private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Kicker kicker;
  private final Indexer indexer;

  private static final Voltage KICKER_FIRE_VOLTAGE = Volts.of(4.0);
  private static final AngularVelocity FLYWHEEL_TOLERANCE = RPM.of(70.0);

  public AutoFireCommand(
      AutoAimCommand autoAim,
      Turret turret,
      Flywheel flywheel,
      Hood hood,
      Kicker kicker,
      Indexer indexer) {
    this.autoAim = autoAim;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.kicker = kicker;
    this.indexer = indexer;

    addRequirements(flywheel, hood, kicker, indexer);
  }

  @Override
  public void execute() {
    // Read the parameters that AutoAimCommand already computed this cycle.
    // This guarantees we use the same turret-offset pose and shoot-on-move
    // correction that the turret is actually aiming at.
    ShotParameters params = autoAim.getLatestParameters();

    // // Guard: AutoAim hasn’t run yet (first cycle or not active)
    // if (params == null) {
    //   kicker.io.setVoltage(Volts.of(0.0));
    //   indexer.io.setRollerVoltage(Volts.of(0.0));
    //   return;
    // }

    boolean turretReady = turret.atSetpoint(TurretConstants.TURRET_TOLERANCE);
    boolean flywheelReady = false;
    boolean hoodReady = true; // Hood tolerance hardcoded until hood.atSetpoint() is implemented
    boolean shotValid = params.isValid();

    if (shotValid) {
      AngularVelocity targetSpeed = RPM.of(params.flywheelSpeedRPM());
      flywheel.io.setWheelVelocitySetpoint(targetSpeed);
      hood.io.setPositionSetpoint(Degrees.of(params.hoodAngleDegrees()));

      flywheelReady = flywheel.atSpeed(targetSpeed, FLYWHEEL_TOLERANCE);
    } else {
      flywheel.io.setWheelVelocitySetpoint(RPM.of(0));
      hood.io.stop();
    }

    boolean readyToFire = turretReady && flywheelReady && hoodReady && shotValid;

    Logger.recordOutput("AutoFire/TurretReady", turretReady);
    Logger.recordOutput("AutoFire/FlywheelReady", flywheelReady);
    Logger.recordOutput("AutoFire/HoodReady", hoodReady);
    Logger.recordOutput("AutoFire/ShotValid", shotValid);
    Logger.recordOutput("AutoFire/ReadyToFire", readyToFire);

    if (readyToFire) {
      kicker.io.setVoltage(KICKER_FIRE_VOLTAGE);
      indexer.io.setRollerVoltage(KICKER_FIRE_VOLTAGE);
    } else {
      kicker.io.setVoltage(Volts.of(0.0));
      indexer.io.setRollerVoltage(Volts.of(0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.io.setWheelVelocitySetpoint(RPM.of(0));
    hood.io.stop();
    kicker.io.setVoltage(Volts.of(0.0));
    indexer.io.setRollerVoltage(Volts.of(0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
