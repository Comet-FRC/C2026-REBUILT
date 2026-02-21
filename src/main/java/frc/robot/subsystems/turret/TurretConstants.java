package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
  public static final int TURRET_MOTOR_ID = 20;
  public static final double GEAR_RATIO = 40.0;
  public static final double TURRET_DIAMETER_INCHES = 6.875;
  public static final double TURRET_CONVERSION_FACTOR = 2 * Math.PI / GEAR_RATIO;

  // soft limits
  public static final Angle MIN_ANGLE = Degrees.of(-90.0);
  public static final Angle MAX_ANGLE = Degrees.of(270.0);

  public static final Angle TURRET_TOLERANCE = Degrees.of(1.0);

  public static final double TURRET_kP = 30.2;
  public static final double TURRET_kI = 0.0;
  public static final double TURRET_kD = 0.0;
  public static final double TURRET_kS = 0.2;
  public static final double TURRET_kV = 4.8;
  public static final double TURRET_kA = 0.0;

  public static final double TURRET_MAX_VELOCITY = 0.5; // Mechanism rotations / second
  public static final double TURRET_MAX_ACCELERATION = 0.25; // Mechanism rotations / second^2

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
