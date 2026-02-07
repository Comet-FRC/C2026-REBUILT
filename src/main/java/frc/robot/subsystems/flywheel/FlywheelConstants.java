package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class FlywheelConstants {
  public static final int FLYWHEEL_LEADER_ID = 15; // these ids most likely need to be changed
  public static final int FLYWHEEL_FOLLOWER_ID = 16;

  public static final Mass WHEEL_MASS = Pounds.of(0.035); // idk value
  public static final Distance WHEEL_RADIUS = Inches.of(1); // idk value

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

  public static final double WHEEL_MOI =
      0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2);
}
