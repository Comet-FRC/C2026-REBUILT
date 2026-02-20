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
    public Angle wheelPosition = Radians.of(0);
    public AngularVelocity wheelVelocity = RadiansPerSecond.of(0);
    public AngularVelocity wheelDesiredVelocity = RadiansPerSecond.of(0);
    public AngularVelocity wheelVelocitySetpoint = RadiansPerSecond.of(0);
    public Voltage wheelAppliedVolts = Volts.of(0);
    public Current wheelSupplyCurrent = Amps.of(0);
    public Temperature wheelMotorTemperature = Celsius.of(0);

    public Angle pivotPosition = Radians.of(0);
    public Angle pivotDesiredPosition = Radians.of(0);
    public AngularVelocity pivotVelocity = RadiansPerSecond.of(0);
    public Voltage pivotRightAppliedVolts = Volts.of(0);
    public Voltage pivotLeftAppliedVolts = Volts.of(0);
    public Current pivotSupplyCurrent = Amps.of(0);
    public Temperature pivotTemperature = Celsius.of(0);

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
