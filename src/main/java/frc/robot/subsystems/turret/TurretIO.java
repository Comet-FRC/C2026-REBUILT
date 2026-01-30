package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public Angle position = Radians.of(0);
    public AngularVelocity velocity = RadiansPerSecond.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current statorCurrent = Amps.of(0);
    public Current supplyCurrent = Amps.of(0);
    public Angle targetPosition = Radians.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set the turret to a target position using MotionMagic. */
  public default void setPosition(Rotation2d position) {}

  /** Set the turret motor voltage directly (open-loop). */
  public default void setVoltage(Voltage volts) {}

  /** Stop the turret motor. */
  public default void stop() {}

  /** Reset the turret encoder to a specific position. */
  public default void resetPosition(Rotation2d position) {}
}
