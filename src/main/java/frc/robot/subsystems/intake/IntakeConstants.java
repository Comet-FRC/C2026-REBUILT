package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class IntakeConstants {
  // Intake wheel motors (SparkMax + NEO)
  public static final int INTAKE_LEADER_ID = 12;
  public static final int INTAKE_FOLLOWER_ID = 23;
  // Intake pivot motors (SparkMax + NEO)
  public static final int PIVOT_RIGHT = 13;
  public static final int PIVOT_LEFT = 14;

  // REV Through Bore Encoder (Duty Cycle) on intake axle
  public static final int THROUGH_BORE_ENCODER_DIO = 0;
  // Offset to calibrate the encoder (in radians)
  // Adjust this so that when the intake is at 90°, the encoder reads 90°
  // Formula: OFFSET = (desired angle in radians) - (raw encoder reading in radians)
  // Formula OFFSET = 1.5708 - throughBoreEncoderPositionRad
  public static final double THROUGH_BORE_ENCODER_OFFSET_RAD = 0.070;

  public static final Angle STOW_ANGLE = Degrees.of(66.0);
  public static final double GEAR_RATIO = 4 * 4 * (38 / 14);

  public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI * (18.0 / 24.0);
  public static final double PIVOT_CONVERSION_FACTOR = 2 * Math.PI * (1.0 / GEAR_RATIO);

  public static final double PIVOT_SIM_kP = 10000;
  public static final double PIVOT_SIM_kI = 0;
  public static final double PIVOT_SIM_kD = 0.01;
  public static final double PIVOT_SIM_kS = 0.3;
  public static final double PIVOT_SIM_kG = 1.0;
  public static final double PIVOT_SIM_kV = 0.069055;
  public static final double PIVOT_SIM_kA = 0.24078;

  // TUNING:
  // SET P I and D to 0
  // MOVE INTAKE TO 45 DEGREES
  // INCREASE kG UNTIL IT HOLDS POSITION
  // SET kS to 0
  //

  public static final double PIVOT_kP = 5.8;
  public static final double PIVOT_kI = 0;
  public static final double PIVOT_kD = 0.41;
  public static final double PIVOT_kS = 0; // Not really used with intakes
  public static final double PIVOT_kG = 0.45;
  public static final double PIVOT_kV = 0.84;
  public static final double PIVOT_kA = 0.10;

  public static final double WHEEL_kP = 0.001;
  public static final double WHEEL_kI = 0;
  public static final double WHEEL_kD = 0;
  public static final double WHEEL_SIM_kP = 0.001;
  public static final double WHEEL_SIM_kI = 0;
  public static final double WHEEL_SIM_kD = 0;
  public static final double WHEEL_kS = 0.15912;
  public static final double WHEEL_kV = 0.021763;
  public static final double WHEEL_kA = 0.0014371;
  public static final double WHEEL_SIM_kS = 0.0003;
  public static final double WHEEL_SIM_kV = 0.01763;
  public static final double WHEEL_SIM_kA = 0.0001;

  public static final Mass WHEEL_MASS = Pounds.of(0.035);
  public static final Distance WHEEL_RADIUS = Inches.of(1);
  public static final double WHEEL_MOI =
      0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2); // 1/2MR^2

  public static final Distance LENGTH = Inches.of(13.423);
  public static final Mass MASS = Pounds.of(5.2);
  public static final double PIVOT_ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
}
