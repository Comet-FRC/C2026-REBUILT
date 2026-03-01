package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
    public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
    public AngularVelocity leftDesiredVelocity = RadiansPerSecond.of(0);
    public AngularVelocity rightDesiredVelocity = RadiansPerSecond.of(0);
    public Angle leftPosition = Radians.of(0);
    public Angle rightPosition = Radians.of(0);
    public AngularVelocity leftVelocitySetpoint = RadiansPerSecond.of(0);
    public AngularVelocity rightVelocitySetpoint = RadiansPerSecond.of(0);
    public Voltage leftAppliedVolts = Volts.of(0);
    public Voltage rightAppliedVolts = Volts.of(0);
    public Current leftSupplyCurrent = Amps.of(0);
    public Current rightSupplyCurrent = Amps.of(0);
    public Temperature leftTemperature = Celsius.of(0);
    public Temperature rightTemperature = Celsius.of(0);
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void stopWheel() {}

  public default void setWheelVoltage(Voltage volts) {}

  public default void setWheelVelocitySetpoint(AngularVelocity velocity) {}

  public default void enabledInit() {}
}
