package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class FlywheelConstants {
  public static final int FLYWHEEL_LEADER_ID = 18;
  public static final int FLYWHEEL_FOLLOWER_ID = 19;

  public static final double GEAR_RATIO = (24.0 / 24.0);

  // Converted from Linear (V / (m/s)) to Rotational (V / (rot/s))
  // Radius = 1.5 inches = 0.0381 meters
  // Circumference = 0.2394 meters
  // Rotational = Linear * Circumference
  public static final double WHEEL_kP = 0.278; // 1.16 * 0.2394
  public static final double WHEEL_kI = 0;
  public static final double WHEEL_kD = 0;

  public static final double WHEEL_kS = 0;
  public static final double WHEEL_kV = 0.124; // 0.52 * 0.2394
  public static final double WHEEL_kA = 0.012; // 0.05 * 0.2394

  public static final double WHEEL_SIM_kP = 0.001;
  public static final double WHEEL_SIM_kI = 0;
  public static final double WHEEL_SIM_kD = 0;
  public static final double WHEEL_SIM_kS = 0.0003;
  public static final double WHEEL_SIM_kV = 0.01763;
  public static final double WHEEL_SIM_kA = 0.0001;

  public static final Mass WHEEL_MASS = Pounds.of(2.27); // 1.14 lbs shooter + 1.13 lbs flywheels
  public static final Distance WHEEL_RADIUS = Inches.of(2.0); // 4" OD wheels
  public static final double WHEEL_MOI = 0.001474; // kg*m^2 (Computed from custom MOIs)
}
