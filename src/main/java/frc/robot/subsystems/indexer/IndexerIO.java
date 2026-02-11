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
    public AngularVelocity rollerVelocity = RadiansPerSecond.of(0);
    public AngularVelocity rollerDesiredVelocity = RadiansPerSecond.of(0);
    public Angle rollerPosition = Radians.of(0);
    public Voltage rollerAppliedVolts = Volts.of(0);
    public Current rollerSupplyCurrent = Amps.of(0);
    public Temperature rollerTemperature = Celsius.of(0);
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void stopRoller() {}

  public default void setRollerVelocitySetpoint(AngularVelocity velocity) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void enabledInit() {}
}
