package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public Angle hoodPosition = Radians.of(0);
    public AngularVelocity hoodVelocity = RadiansPerSecond.of(0);
    public Angle hoodDesiredPosition = Radians.of(0);
    public Angle hoodPositionSetpoint = Radians.of(0);
    public Voltage hoodAppliedVolts = Volts.of(0);
    public Current hoodSupplyCurrent = Amps.of(0);
    public Temperature hoodTemperature = Celsius.of(0);
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void stop() {}

  public default void setVoltage(Voltage volts) {}

  public default void setPositionSetpoint(Angle position) {}

  public default void enabledInit() {}

  public default void resetPosition(Angle position) {}
}
