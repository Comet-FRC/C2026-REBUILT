package frc.robot.subsystems.indexer;

public final class IndexerConstants {
  public static final int INDEXER_MOTOR_ID = 15;

  // MAX Planetary 4:1 reduction
  public static final double GEAR_RATIO = 4.0;
  public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI * 1 / GEAR_RATIO;

  // Wheel moment of inertia (1.250" diameter, 0.170 lbs per wheel)
  public static final double WHEEL_MOMENT_OF_INERTIA = 9.71e-6;

  // PID gains for roller speed control
  public static final double ROLLER_kP = 0.001;
  public static final double ROLLER_kI = 0;
  public static final double ROLLER_kD = 0;
  public static final double ROLLER_SIM_kP = 3.596;
  public static final double ROLLER_SIM_kI = 0;
  public static final double ROLLER_SIM_kD = 0;
  public static final double ROLLER_kS = 0.15912;
  public static final double ROLLER_kV = 0.021763;
  public static final double ROLLER_kA = 0.0014371;
  public static final double ROLLER_SIM_kS = 6.6332;
  public static final double ROLLER_SIM_kV = 1688.8;
  public static final double ROLLER_SIM_kA = 0;
}
