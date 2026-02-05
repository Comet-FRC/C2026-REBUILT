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
    public AngularVelocity rollerVelocity = RadiansPerSecond.of(0);
    public AngularVelocity rollerDesiredVelocity = RadiansPerSecond.of(0);
    public Angle rollerPosition = Radians.of(0);
    public Voltage rollerAppliedVolts = Volts.of(0);
    public Current rollerSupplyCurrent = Amps.of(0);
    public Temperature rollerTemperature = Celsius.of(0);
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void stopWheel() {}

  public default void enabledInit() {}
}
