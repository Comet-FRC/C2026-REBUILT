package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class TurretConstants {
  public static final int TURRET_MOTOR_ID = 20;
  public static final double GEAR_RATIO = 40.0;
  public static final double TURRET_DIAMETER_INCHES = 6.875;
  public static final double TURRET_CONVERSION_FACTOR = 2 * Math.PI / GEAR_RATIO;

  public static final Distance PHYSICAL_OFFSET = Inches.of(3.75);

  // soft limits
  public static final Angle MIN_ANGLE = Degrees.of(-32.0);
  public static final Angle MAX_ANGLE = Degrees.of(330.0);

  public static final Angle TURRET_TOLERANCE = Degrees.of(4.0);

  public static final double TURRET_kP = 30.2;
  public static final double TURRET_kI = 0.0;
  public static final double TURRET_kD = 0.0;
  public static final double TURRET_kS = 0.2;
  public static final double TURRET_kV = 4.8;
  public static final double TURRET_kA = 0.0;

  public static final double TURRET_MAX_VELOCITY = 2.0; // Mechanism rotations / second
  public static final double TURRET_MAX_ACCELERATION = 4; // Mechanism rotations / second^2

  // Simulation PID gains
  public static final double TURRET_SIM_kP = 5.0;
  public static final double TURRET_SIM_kI = 0.0;
  public static final double TURRET_SIM_kD = 0.1;
  public static final double TURRET_SIM_kS = 0.04;
  public static final double TURRET_SIM_kV = 0.5;
  public static final double TURRET_SIM_kA = 0.0;

  // Moment of inertia estimate (adjust based on actual turret mass/dimensions)
  public static final double TURRET_MOI = 0.05; // kg*m^2

  // ── Absolute Encoders (REV Throughbore via DIO) ─────────────────────────────
  // 19T and 21T encoder gears meshed against the 200T turret ring.
  // DIO port assignments
  public static final int ENCODER_19T_DIO_PORT = 3;
  public static final int ENCODER_21T_DIO_PORT = 2;

  public static final double ENCODER_19T_OFFSET = 0.111;
  public static final double ENCODER_21T_OFFSET = 0.538;

  // Set to true if that encoder counts DOWN when the turret moves in its positive direction.
  // Check: rotate turret positive, watch Raw19T and Raw21T in AdvantageScope.
  // Both should move in the same direction. Invert whichever one doesn't.
  public static final boolean IS_19T_INVERTED = false;
  public static final boolean IS_21T_INVERTED = false;

  public static final int NUM_TEETH_GEAR_1 = 19;
  public static final int NUM_TEETH_GEAR_2 = 21;
  public static final int NUM_TEETH_TURRET = 200;
}
