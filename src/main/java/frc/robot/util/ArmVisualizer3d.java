package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

/**
 * Visualizer for 3D arm/pivot mechanisms in AdvantageScope. Publishes Pose3d data that can be used
 * with AdvantageScope's 3D field view and custom robot models with articulated components.
 */
public class ArmVisualizer3d {
  private final String name;
  private final Translation3d position;
  private final Angle angleOffset;

  private Angle pivotAngle;

  /**
   * Creates a new ArmVisualizer3d.
   *
   * @param name The name of the mechanism (used as the log key)
   * @param position The position of the pivot point relative to the robot origin (in meters)
   * @param angleOffset Offset to add to the angle for visual alignment (e.g., if 90° in code should
   *     show as straight up but currently doesn't, adjust this offset)
   */
  public ArmVisualizer3d(String name, Translation3d position, Angle angleOffset) {
    this.name = name;
    this.position = position;
    this.angleOffset = angleOffset;
  }

  /**
   * Sets the current angle of the arm/pivot mechanism.
   *
   * @param angle The current angle of the mechanism
   */
  public void setArmAngle(Angle angle) {
    pivotAngle = angle;
    Logger.recordOutput("Mechanism3D/" + this.name + "Angle", angle);
  }

  /** Publishes the mechanism pose to NetworkTables for AdvantageScope visualization. */
  public void publish() {
    Logger.recordOutput("Mechanism3D/" + this.name, getPose());
  }

  /**
   * Gets the current Pose3d for this mechanism. Use this to build component pose arrays for
   * AdvantageScope custom robots.
   *
   * @return The current Pose3d of the mechanism
   */
  public Pose3d getPose() {
    // Apply angle offset for visual alignment, negate to match rotation direction
    double adjustedAngle = -pivotAngle.in(Radians) + angleOffset.in(Radians);

    return new Pose3d(
        position,
        new Rotation3d(
            0, // Roll (X)
            adjustedAngle, // Pitch (Y) - the pivot angle with offset
            0 // Yaw (Z)
            ));
  }
}
