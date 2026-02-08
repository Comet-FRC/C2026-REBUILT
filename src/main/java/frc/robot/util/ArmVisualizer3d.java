package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final Rotation2d forwardDirection;

  private Angle pivotAngle;

  /**
   * Creates a new ArmVisualizer3d.
   *
   * @param name The name of the mechanism (used as the log key)
   * @param position The position of the pivot point relative to the robot origin (in meters)
   * @param forwardDirection The direction the mechanism points when at zero angle
   */
  public ArmVisualizer3d(String name, Translation3d position, Rotation2d forwardDirection) {
    this.name = name;
    this.position = position;
    this.forwardDirection = forwardDirection;
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
    // Position the pivot at the configured location
    // Apply the pivot angle as rotation around the Y axis (pitch) for intake mechanisms
    // The config.json zeroedRotations handle orienting the model correctly
    return new Pose3d(
        position,
        new Rotation3d(
            0, // Roll (X)
            pivotAngle.in(Radians), // Pitch (Y) - the pivot angle
            forwardDirection.getRadians() // Yaw (Z) - direction facing
            ));
  }
}
