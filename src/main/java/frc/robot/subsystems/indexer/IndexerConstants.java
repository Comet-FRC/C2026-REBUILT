package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class IndexerConstants {

  public static final int MOTOR_ID = 13;

  public static final double WHEEL_CONVERSION_FACTOR =
      2 * Math.PI * (15.0 / 18.0); // TODO: CHANGE PULLEY RATIO

  // TODO: Does this actually apply here?

  public static final Distance WHEEL_RADIUS = Inches.of(3);
  public static final Mass WHEEL_MASS = Pounds.of(0.86);
  public static final double WHEEL_MOI =
      0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2);

  public static final double kP = 0.01;
  public static final double kI = 0.000004;
  public static final double kD = 10;
  public static final double kS = 0.048874;
  public static final double kV = 0.04;
  public static final double kA = 0.011556;

  public static final double SIM_kP = 0.65;
  public static final double SIM_kI = 0.01;
  public static final double SIM_kD = 0;
  public static final double SIM_kS = 0;
  public static final double SIM_kV = 0.1;
  public static final double SIM_kA = 0.00094517;

  public static final AngularVelocity ACCEPTABLE_VELOCITY_ERROR = RPM.of(10);
}
