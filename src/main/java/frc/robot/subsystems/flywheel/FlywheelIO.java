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
    public AngularVelocity wheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity wheelDesiredVelocity = RadiansPerSecond.of(0);
    public Angle wheelPosition = Radians.of(0);
    public AngularVelocity wheelVelocitySetpoint = RadiansPerSecond.of(0);
    public Voltage wheelAppliedVolts = Volts.of(0);
    public Current wheelSupplyCurrent = Amps.of(0);
    public Temperature wheelTemperature = Celsius.of(0);
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void stopWheel() {}

  public default void setWheelVoltage(Voltage volts) {}

  public default void setWheelVelocitySetpoint(AngularVelocity velocity) {}

  public default void enabledInit() {}
}
