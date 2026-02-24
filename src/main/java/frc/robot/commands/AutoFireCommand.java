package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Bind this to a button. While held, it watches the turret and flywheel — the moment both are
 * locked on and up to speed, it feeds the kicker to fire. Stops feeding as soon as either drops out
 * of tolerance.
 */
public class AutoFireCommand extends Command {
  private final Turret turret;
  private final Flywheel flywheel;
  private final Kicker kicker;
  private final Supplier<ShotParameters> parametersSupplier;

  private static final Voltage KICKER_FIRE_VOLTAGE = Volts.of(4.0);
  private static final AngularVelocity FLYWHEEL_TOLERANCE = RPM.of(100.0);

  public AutoFireCommand(
      Turret turret,
      Flywheel flywheel,
      Kicker kicker,
      Supplier<ShotParameters> parametersSupplier) {
    this.turret = turret;
    this.flywheel = flywheel;
    this.kicker = kicker;
    this.parametersSupplier = parametersSupplier;

    addRequirements(kicker);
  }

  @Override
  public void execute() {
    ShotParameters params = parametersSupplier.get();

    boolean turretReady = turret.atSetpoint(TurretConstants.TURRET_TOLERANCE);
    boolean flywheelReady = false;
    boolean shotValid = false;

    if (params != null) {
      shotValid = params.isValid();
      AngularVelocity targetSpeed = RPM.of(params.flywheelSpeedRPM());
      flywheelReady = flywheel.atSpeed(targetSpeed, FLYWHEEL_TOLERANCE);
    }

    boolean readyToFire = turretReady && flywheelReady && shotValid;

    Logger.recordOutput("AutoFire/TurretReady", turretReady);
    Logger.recordOutput("AutoFire/FlywheelReady", flywheelReady);
    Logger.recordOutput("AutoFire/ShotValid", shotValid);
    Logger.recordOutput("AutoFire/ReadyToFire", readyToFire);

    if (readyToFire) {
      kicker.io.setVoltage(KICKER_FIRE_VOLTAGE);
    } else {
      kicker.io.setVoltage(Volts.of(0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    kicker.io.setVoltage(Volts.of(0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
