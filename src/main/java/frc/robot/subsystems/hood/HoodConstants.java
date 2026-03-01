package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class HoodConstants {
  // NEO motor ID (SparkMax)
  public static final int HOOD_MOTOR_ID = 21;

  // Gear train: 18T → 60T → 10T → 164T
  // Stage 1: 60/18 = 3.333...
  // Stage 2: 164/10 = 16.4
  // Total: 3.333 × 16.4 = 54.667:1
  public static final double GEAR_RATIO = (60.0 / 18.0) * (164.0 / 10.0);

  public static final Angle MIN_ANGLE = Degrees.of(0.0);
  public static final Angle MAX_ANGLE = Degrees.of(30.0);

  // PID gains for hood position control (NEEDS TUNING)
  public static final double HOOD_kP = 25;
  public static final double HOOD_kI = 0.0;
  public static final double HOOD_kD = 0.0;
  public static final double HOOD_kS = 0.4;
  public static final double HOOD_kV = 0.8;
  public static final double HOOD_kA = 0.05;

  // Simulation PID gains
  public static final double HOOD_SIM_kP = 5.0;
  public static final double HOOD_SIM_kI = 0.0;
  public static final double HOOD_SIM_kD = 0.1;
  public static final double HOOD_SIM_kS = 0.0;
  public static final double HOOD_SIM_kV = 0.5;
  public static final double HOOD_SIM_kA = 0.0;

  // Moment of inertia estimate (adjust based on actual hood mass/dimensions)
  public static final double HOOD_MOI = 0.01; // kg*m^2 — small since it's just the hood plate
}
