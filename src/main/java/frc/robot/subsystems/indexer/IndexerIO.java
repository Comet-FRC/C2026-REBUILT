package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public AngularVelocity WheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity WheelDesiredVelocity = RadiansPerSecond.of(0);
    public Angle WheelPosition = Radians.of(0);
    public Voltage WheelAppliedVoltage = Volts.of(0);
    public Current WheelSupplyCurrent = Amps.of(0);
    public Temperature WheelTemperature = Celsius.of(0);
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Sets the desired velocity of the indexer. */
  public default void setWheelVelocitySetpoint(AngularVelocity velocity) {}
  /** Sets the desired voltage of the indexer. */
  public default void setVoltage(Voltage volts) {}

  public default void stop() {}
}
