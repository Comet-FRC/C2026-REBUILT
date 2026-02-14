package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
  // Kraken X60 motor ID
  public static final int TURRET_MOTOR_ID = 20;

  // Gear ratio: 4:1 gearbox × 10:1 turret (20T pinion driving 200T gear) = 40:1 total
  public static final double GEAR_RATIO = 40.0;

  // Turret diameter: 6.875 inches
  public static final double TURRET_DIAMETER_INCHES = 6.875;

  // Manual control voltage
  public static final double MANUAL_CONTROL_VOLTAGE = 3.0;

  // Conversion factor: motor rotations to turret radians
  public static final double TURRET_CONVERSION_FACTOR = 2 * Math.PI / GEAR_RATIO;

  // Soft limits for turret rotation (adjust based on physical limits)
  public static final Angle MIN_ANGLE = Degrees.of(-180.0);
  public static final Angle MAX_ANGLE = Degrees.of(180.0);

  // Tolerance for "at setpoint" checks (NEEDS TUNING)
  public static final Angle TURRET_TOLERANCE = Degrees.of(2.0);

  // PID gains for turret position control (NEEDS TUNING)
  public static final double TURRET_kP = 0.0;
  public static final double TURRET_kI = 0.0;
  public static final double TURRET_kD = 0.0;
  public static final double TURRET_kS = 0.0;
  public static final double TURRET_kV = 0.0;
  public static final double TURRET_kA = 0.0;

  // Simulation PID gains
  public static final double TURRET_SIM_kP = 5.0;
  public static final double TURRET_SIM_kI = 0.0;
  public static final double TURRET_SIM_kD = 0.1;
  public static final double TURRET_SIM_kS = 0.0;
  public static final double TURRET_SIM_kV = 0.5;
  public static final double TURRET_SIM_kA = 0.0;

  // Moment of inertia estimate (adjust based on actual turret mass/dimensions)
  public static final double TURRET_MOI = 0.05; // kg*m^2
}
