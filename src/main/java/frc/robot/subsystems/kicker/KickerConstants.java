package frc.robot.subsystems.kicker;

public final class KickerConstants {
  public static final int LEFT_MOTOR_ID = 16;
  public static final int RIGHT_MOTOR_ID = 17;

  // MAX Planetary 4:1 reduction, 18T to 18T (1:1 external)
  public static final double GEAR_RATIO = 4.0;
  public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI * 1 / GEAR_RATIO;

  // Wheel moment of inertia (estimate, adjust as needed)
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
