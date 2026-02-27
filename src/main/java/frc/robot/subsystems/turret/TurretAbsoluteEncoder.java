package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Computes the turret's absolute angle using two REV Throughbore encoders on intermediate gears
 * (19T and 21T) meshed against the 400T turret ring gear.
 *
 * <p><b>Chinese Remainder Theorem (CRT) approach:</b><br>
 * As the turret rotates by {@code n} teeth, the 19T encoder completes {@code n/19} revolutions and
 * the 21T encoder completes {@code n/21} revolutions. The pair {@code (n mod 19, n mod 21)} is
 * unique for any 399 consecutive turret tooth positions (since gcd(19,21)=1 and lcm(19,21)=399).
 * The turret's actual travel range spans 360 teeth, well within this unambiguous window.
 *
 * <p><b>CRT formula:</b><br>
 * {@code n = (a * 210 + b * 190) mod 399}<br>
 * where {@code a = round(adj19T * 19) mod 19} and {@code b = round(adj21T * 21) mod 21}<br>
 * Coefficients: {@code 21 * inv(21, 19) = 21 * 10 = 210} and {@code 19 * inv(19, 21) = 19 * 10 =
 * 190}
 *
 * <p><b>Offset calibration:</b><br>
 * Set {@link TurretConstants#ENCODER_19T_OFFSET} and {@link TurretConstants#ENCODER_21T_OFFSET} by
 * manually positioning the turret at exactly 180° (center), reading the raw encoder values from
 * AdvantageScope under "Turret/AbsEncoder/Raw19T" and "Turret/AbsEncoder/Raw21T", and storing those
 * values as the offsets.
 */
public class TurretAbsoluteEncoder {

  // --- Gear constants ---
  private static final int TEETH_19T = 19;
  private static final int TEETH_21T = 21;
  private static final int TURRET_TEETH = 200;

  // lcm(19, 21) = 399 — the period of the combined encoder signal
  private static final int CRT_PERIOD = TEETH_19T * TEETH_21T; // 399

  // CRT coefficients (precomputed):
  //   inv(21 mod 19 = 2, mod 19) = 10  →  coeff_19T = 21 * 10 = 210
  //   inv(19 mod 21 =-2, mod 21) = 10  →  coeff_21T = 19 * 10 = 190
  private static final int CRT_COEFF_19T = 210;
  private static final int CRT_COEFF_21T = 190;

  // --- Hardware ---
  private final DutyCycleEncoder encoder19T;
  private final DutyCycleEncoder encoder21T;

  // Last computed absolute angle, cached for telemetry
  private double lastAbsoluteAngleDeg = 0.0;
  private double lastRaw19T = 0.0;
  private double lastRaw21T = 0.0;

  public TurretAbsoluteEncoder() {
    encoder19T = new DutyCycleEncoder(TurretConstants.ENCODER_19T_DIO_PORT);
    encoder21T = new DutyCycleEncoder(TurretConstants.ENCODER_21T_DIO_PORT);
  }

  /**
   * Computes and returns the turret's absolute angle in degrees.
   *
   * <p>This should be called once at robot enable / startup to seed the TalonFX internal encoder.
   * It can also be called periodically for telemetry.
   *
   * @return turret angle in degrees, in the range [0°, 360°)
   */
  public double getAbsoluteAngleDegrees() {
    // Extract the strict [0, 1) fractional revolution, stripping off full accumulator rotations
    lastRaw19T = (encoder19T.get() % 1.0 + 1.0) % 1.0;
    lastRaw21T = (encoder21T.get() % 1.0 + 1.0) % 1.0;

    // Apply inversion (flip direction if encoder counts backwards relative to turret)
    double effective19T =
        TurretConstants.ENCODER_19T_INVERTED ? (1.0 - lastRaw19T) % 1.0 : lastRaw19T;
    double effective21T =
        TurretConstants.ENCODER_21T_INVERTED ? (1.0 - lastRaw21T) % 1.0 : lastRaw21T;

    // Apply per-encoder offsets
    double adj19T = (effective19T - TurretConstants.ENCODER_19T_OFFSET) % 1.0;
    if (adj19T < 0) adj19T += 1.0;

    double adj21T = (effective21T - TurretConstants.ENCODER_21T_OFFSET) % 1.0;
    if (adj21T < 0) adj21T += 1.0;

    // Convert fractional encoder rotation to turret tooth remainder
    int a = (int) Math.round(adj19T * TEETH_19T) % TEETH_19T;
    int b = (int) Math.round(adj21T * TEETH_21T) % TEETH_21T;

    // CRT: reconstruct turret tooth index n in [0, 398]
    int nCrt = (a * CRT_COEFF_19T + b * CRT_COEFF_21T) % CRT_PERIOD;
    if (nCrt < 0) nCrt += CRT_PERIOD;

    /*
     * GEAR BACKLASH FILTER:
     * If the encoders are slightly mismatched due to physical backlash, the `Math.round` inputs
     * might sit on opposite sides of a rounding boundary (e.g., 2.49 rounds to 2, but 3.51 rounds to 4).
     * This causes the CRT math to evaluate to completely incorrect sequences (a 60-degree jump).
     *
     * Therefore, we test the closest 3 teeth mathematically: n-1, n, and n+1. We select the tooth
     * that is closest to what the continuous 19T Fractional Encoder claims it should be.
     */
    int bestN = nCrt;
    double minError = Double.MAX_VALUE;

    for (int offset = -1; offset <= 1; offset++) {
      int testN = (nCrt + offset) % CRT_PERIOD;
      if (testN < 0) testN += CRT_PERIOD;

      // What should the 19T encoder fraction strictly read if testN were the true tooth?
      double expected19T = (testN / (double) TEETH_19T) % 1.0;

      // Compute circular distance between expected 19T fraction and actual 19T fraction
      double dist = Math.abs(expected19T - adj19T);
      if (dist > 0.5) dist = 1.0 - dist;

      if (dist < minError) {
        minError = dist;
        bestN = testN;
      }
    }

    // Convert robust tooth index to degrees: bestN teeth / 200 teeth * 360°
    double rawAngleDeg = (bestN / (double) TURRET_TEETH) * 360.0;

    lastAbsoluteAngleDeg = (rawAngleDeg + 180) % 360;
    return lastAbsoluteAngleDeg;
  }

  /** Returns true if both encoders are physically connected and outputting valid signals. */
  public boolean isConnected() {
    return encoder19T.isConnected() && encoder21T.isConnected();
  }

  // --- Telemetry getters (populate TurretIOInputs each loop) ---

  /** Raw fractional position of the 19T encoder [0, 1). */
  public double getRaw19T() {
    return lastRaw19T;
  }

  /** Raw fractional position of the 21T encoder [0, 1). */
  public double getRaw21T() {
    return lastRaw21T;
  }

  /**
   * Last computed absolute angle in degrees (cached from most recent {@link
   * #getAbsoluteAngleDegrees()} call).
   */
  public double getLastAbsoluteAngleDeg() {
    return lastAbsoluteAngleDeg;
  }
}
