package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretAbsoluteEncoder {
  private final double[] POSSIBLE_19T_VALUES = new double[TurretConstants.NUM_TEETH_GEAR_2];
  private final double[] POSSIBLE_21T_VALUES = new double[TurretConstants.NUM_TEETH_GEAR_1];

  private final DutyCycleEncoder ENCODER_GEAR_1;
  private final DutyCycleEncoder ENCODER_GEAR_2;

  private Angle lastAbsoluteAngle = Degrees.zero();

  public TurretAbsoluteEncoder() {
    ENCODER_GEAR_1 = new DutyCycleEncoder(TurretConstants.ENCODER_19T_DIO_PORT);
    ENCODER_GEAR_2 = new DutyCycleEncoder(TurretConstants.ENCODER_21T_DIO_PORT);
  }

  /**
   * Computes and returns the turret's absolute angle in degrees.
   *
   * <p>This should be called once at robot enable / startup to seed the TalonFX internal encoder.
   * It can also be called periodically for telemetry.
   *
   * @return turret angle in degrees, in the range [-180°, 180°)
   */
  public Angle getAngle() {
    double position_19T = getRaw19T(); // [0, 1) — fractional encoder revolution
    double position_21T = getRaw21T();

    // Apply inversion (flip direction if encoder counts backwards relative to turret)
    position_19T = TurretConstants.IS_19T_INVERTED ? (1.0 - position_19T) % 1.0 : position_19T;
    position_21T = TurretConstants.IS_21T_INVERTED ? (1.0 - position_21T) % 1.0 : position_21T;

    // Apply per-encoder offsets (shift so that turret reference position reads as the correct
    // angle)
    position_19T = (position_19T - TurretConstants.ENCODER_19T_OFFSET + 1.0) % 1.0;
    position_21T = (position_21T - TurretConstants.ENCODER_21T_OFFSET + 1.0) % 1.0;

    for (int n = 0; n < TurretConstants.NUM_TEETH_GEAR_2; ++n) {
      double ratio = (double) TurretConstants.NUM_TEETH_GEAR_1 / TurretConstants.NUM_TEETH_TURRET;
      double encoderRotations = n + position_19T;
      this.POSSIBLE_19T_VALUES[n] = ratio * encoderRotations;
    }
    for (int n = 0; n < TurretConstants.NUM_TEETH_GEAR_1; ++n) {
      double ratio = (double) TurretConstants.NUM_TEETH_GEAR_2 / TurretConstants.NUM_TEETH_TURRET;
      double encoderRotations = n + position_21T;
      this.POSSIBLE_21T_VALUES[n] = ratio * encoderRotations;
    }

    // now we select the correct rotation value
    double minDiff = Double.MAX_VALUE;
    int min19TIndex = 0;
    int min21TIndex = 0;
    for (int i = 0; i < POSSIBLE_19T_VALUES.length; ++i) {
      for (int j = 0; j < POSSIBLE_21T_VALUES.length; ++j) {
        double diff = Math.abs(POSSIBLE_19T_VALUES[i] - POSSIBLE_21T_VALUES[j]);
        if (diff < minDiff) {
          minDiff = diff;
          min19TIndex = i;
          min21TIndex = j;
        }
      }
    }

    double guess_sum = POSSIBLE_19T_VALUES[min19TIndex] + POSSIBLE_21T_VALUES[min21TIndex];
    double best_rotation_guess = guess_sum / 2.0;

    Angle turretAngle = Degrees.of(360.0 * best_rotation_guess - 180.0);
    this.lastAbsoluteAngle = turretAngle;
    return turretAngle;
  }

  public double getRaw19T() {
    return ENCODER_GEAR_1.get();
  }

  public double getRaw21T() {
    return ENCODER_GEAR_2.get();
  }

  public double getLastAbsoluteAngleDeg() {
    return lastAbsoluteAngle.in(Degrees);
  }

  /** Returns true if both encoders are physically connected and outputting valid signals. */
  public boolean isConnected() {
    return ENCODER_GEAR_1.isConnected() && ENCODER_GEAR_2.isConnected();
  }
}
