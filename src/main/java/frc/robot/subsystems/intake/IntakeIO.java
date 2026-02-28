package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Angle LeftwheelPosition = Radians.of(0);
    public AngularVelocity LeftwheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity LeftwheelDesiredVelocity = RadiansPerSecond.of(0);
    public AngularVelocity LeftwheelVelocitySetpoint = RadiansPerSecond.of(0);
    public Voltage LeftwheelAppliedVolts = Volts.of(0);
    public Current LeftwheelSupplyCurrent = Amps.of(0);
    public Temperature LeftwheelMotorTemperature = Celsius.of(0);

    public Angle RightwheelPosition = Radians.of(0);
    public AngularVelocity RightwheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity RightwheelDesiredVelocity = RadiansPerSecond.of(0);
    public AngularVelocity RightwheelVelocitySetpoint = RadiansPerSecond.of(0);
    public Voltage RightwheelAppliedVolts = Volts.of(0);
    public Current RightwheelSupplyCurrent = Amps.of(0);
    public Temperature RightwheelMotorTemperature = Celsius.of(0);

    public Angle pivotPosition = Radians.of(0);
    public Angle pivotDesiredPosition = Radians.of(0);
    public AngularVelocity pivotVelocity = RadiansPerSecond.of(0);
    public Voltage pivotRightAppliedVolts = Volts.of(0);
    public Voltage pivotLeftAppliedVolts = Volts.of(0);
    public Current pivotRightSupplyCurrent = Amps.of(0);
    public Current pivotLeftSupplyCurrent = Amps.of(0);
    public Temperature pivotRightTemperature = Celsius.of(0);
    public Temperature pivotLeftTemperature = Celsius.of(0);

    // REV Through Bore Encoder (Duty Cycle)
    public double throughBoreEncoderPositionRaw = 0.0; // Value 0-1 for one full rotation
    public boolean throughBoreEncoderConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void stopWheel() {}

  public default void stopPivot() {}

  public default void setWheelVelocitySetpoint(AngularVelocity velocity) {}

  public default void setWheelVoltage(Voltage volts) {}

  public default void setPivotPositionSetpoint(Angle position) {}

  public default void setPivotVoltage(Voltage volts) {}

  public default void enabledInit() {}
}
