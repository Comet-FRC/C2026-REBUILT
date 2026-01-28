package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    // Left motor
    public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
    public AngularVelocity leftDesiredVelocity = RadiansPerSecond.of(0);
    public Angle leftPosition = Radians.of(0);
    public Voltage leftAppliedVolts = Volts.of(0);
    public Current leftSupplyCurrent = Amps.of(0);
    public Temperature leftTemperature = Celsius.of(0);

    // Right motor
    public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
    public AngularVelocity rightDesiredVelocity = RadiansPerSecond.of(0);
    public Angle rightPosition = Radians.of(0);
    public Voltage rightAppliedVolts = Volts.of(0);
    public Current rightSupplyCurrent = Amps.of(0);
    public Temperature rightTemperature = Celsius.of(0);
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void stop() {}

  public default void setVelocitySetpoint(AngularVelocity velocity) {}

  public default void setVoltage(Voltage volts) {}

  public default void enabledInit() {}
}
