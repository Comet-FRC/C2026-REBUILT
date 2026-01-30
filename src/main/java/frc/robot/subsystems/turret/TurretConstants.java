package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {
  // CAN IDs - UPDATE THESE TO MATCH YOUR HARDWARE
  public static final int TURRET_MOTOR_ID = 20;

  // Gear ratio from motor to turret output
  // Calculate as: (gearbox ratio) * (turret gear teeth / motor pinion teeth)
  // Example: If gearbox is 5:1 and 20T pinion drives turret, and turret gear has 200T:
  //          Gear ratio = 5 * (200/20) = 50:1
  // UPDATE THIS VALUE based on your actual mechanism
  public static final double GEAR_RATIO = 10.0;

  // Turret offset from robot forward
  // The turret faces 180° opposite of the intake when starting
  public static final Rotation2d TURRET_STARTING_OFFSET = Rotation2d.fromDegrees(180.0);

  // Soft limits (in rotations) - set to null if no limits
  // Positive = counterclockwise, Negative = clockwise when viewed from above
  public static final Double FORWARD_SOFT_LIMIT = null; // e.g., 0.5 for 180° limit
  public static final Double REVERSE_SOFT_LIMIT = null; // e.g., -0.5 for 180° limit

  // MotionMagic constraints
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0; // rotations per second
  public static final double MOTION_MAGIC_ACCELERATION = 4.0; // rotations per second squared
  public static final double MOTION_MAGIC_JERK = 40.0; // rotations per second cubed

  // PID gains for position control
  public static final Slot0Configs TURRET_GAINS =
      new Slot0Configs()
          .withKP(50.0) // Proportional gain
          .withKI(0.0) // Integral gain
          .withKD(0.5) // Derivative gain
          .withKS(0.1) // Static friction compensation
          .withKV(0.12) // Velocity feed-forward
          .withKA(0.0); // Acceleration feed-forward

  // Simulation constants
  public static final double SIM_MOI = 0.05; // Moment of inertia in kg*m^2

  // Current limits
  public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps
  public static final double SUPPLY_CURRENT_LIMIT = 30.0; // Amps
}
