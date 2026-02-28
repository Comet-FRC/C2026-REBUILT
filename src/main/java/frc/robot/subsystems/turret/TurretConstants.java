package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
  public static final int TURRET_MOTOR_ID = 20;
  public static final double GEAR_RATIO = 40.0;
  public static final double TURRET_DIAMETER_INCHES = 6.875;
  public static final double TURRET_CONVERSION_FACTOR = 2 * Math.PI / GEAR_RATIO;

  // soft limits
  public static final Angle MIN_ANGLE = Degrees.of(0.0);
  public static final Angle MAX_ANGLE = Degrees.of(360.0);

  public static final Angle TURRET_TOLERANCE = Degrees.of(4.0);

  public static final double TURRET_kP = 30.2;
  public static final double TURRET_kI = 0.0;
  public static final double TURRET_kD = 0.0;
  public static final double TURRET_kS = 0.2;
  public static final double TURRET_kV = 4.8;
  public static final double TURRET_kA = 0.0;

  public static final double TURRET_MAX_VELOCITY = 13.3; // Mechanism rotations / second
  public static final double TURRET_MAX_ACCELERATION = 30; // Mechanism rotations / second^2

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
  // 19T and 21T encoder gears meshed against the 400T turret ring.
  // DIO port assignments
  public static final int ENCODER_19T_DIO_PORT = 3;
  public static final int ENCODER_21T_DIO_PORT = 2;

  // Per-encoder zero offsets (fractional rotations, [0, 1)).
  // ── HOW TO CALIBRATE ────────────────────────────────────────────────────────
  //  1. Deploy code with offsets set to 0.0.
  //  2. Manually position the turret to exactly 180° (dead center / forward).
  //  3. Open AdvantageScope → Turret/AbsEncoder/Raw19T and Raw21T.
  //  4. Record the displayed raw values and paste them below.
  //  5. Redeploy — the CRT solver will now return 180° at that position.
  public static final double ENCODER_19T_OFFSET = 0.775;
  public static final double ENCODER_21T_OFFSET = 0.694;

  // Set to true if that encoder counts DOWN when the turret moves in its positive direction.
  // Check: rotate turret positive, watch Raw19T and Raw21T in AdvantageScope.
  // Both should move in the same direction. Invert whichever one doesn't.
  public static final boolean IS_19T_INVERTED = false;
  public static final boolean IS_21T_INVERTED = false;

  public static final int NUM_TEETH_GEAR_1 = 19;
  public static final int NUM_TEETH_GEAR_2 = 21;
  public static final int NUM_TEETH_TURRET = 200;
}
