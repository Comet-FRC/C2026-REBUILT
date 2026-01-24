package frc.robot.subsystems.Kicker;

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
    public AngularVelocity topWheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity topWheelDesiredVelocity = RadiansPerSecond.of(0);
    public Angle topWheelPosition = Radians.of(0);
    public Voltage topWheelAppliedVoltage = Volts.of(0);
    public Current topWheelSupplyCurrent = Amps.of(0);
    public Temperature topTemperature = Celsius.of(0);

    public AngularVelocity bottomWheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity bottomWheelDesiredVelocity = RadiansPerSecond.of(0);
    public Angle bottomWheelPosition = Radians.of(0);
    public Voltage bottomWheelAppliedVoltage = Volts.of(0);
    public Current bottomWheelSupplyCurrent = Amps.of(0);
    public Temperature bottomTemperature = Celsius.of(0);
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void stop() {}

  public default void setWheelVelocitySetpoint(
      AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    this.setTopVelocitySetpoint(topVelocity);
    this.setBottomVelocitySetpoint(bottomVelocity);
  }

  public default void setTopVelocitySetpoint(AngularVelocity velocity) {}

  public default void setBottomVelocitySetpoint(AngularVelocity velocity) {}

  public default void setTopVoltage(Voltage volts) {}

  public default void setBottomVoltage(Voltage volts) {}
}
