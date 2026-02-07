package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public Angle turretPosition = Radians.of(0);
    public AngularVelocity turretVelocity = RadiansPerSecond.of(0);
    public Angle turretDesiredPosition = Radians.of(0);
    public Angle turretPositionSetpoint = Radians.of(0);
    public Voltage turretAppliedVolts = Volts.of(0);
    public Current turretSupplyCurrent = Amps.of(0);
    public Temperature turretTemperature = Celsius.of(0);
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void stop() {}

  public default void setVoltage(Voltage volts) {}

  public default void setPositionSetpoint(Angle position) {}

  public default void enabledInit() {}

  public default void resetPosition(Angle position) {}
}
