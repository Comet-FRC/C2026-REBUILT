// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 = // left camera
      new Transform3d(
          new Translation3d(Inches.of(-9.944988), Inches.of(9.848857), Inches.of(7.862498)),
          new Rotation3d(Degrees.of(0), Degrees.of(-25), Degrees.of(100)));
  public static Transform3d robotToCamera1 = // right camera
      new Transform3d(
          new Translation3d(Inches.of(-9.944988), Inches.of(-9.848857), Inches.of(7.862498)),
          new Rotation3d(Degrees.of(0), Degrees.of(-25), Degrees.of(200)));
  public static Transform3d robotTolimelightCamera = // limelight Transform3d created
      new Transform3d(
          new Translation3d(
              Inches.of(-9.944988),
              Inches.of(-9.848857),
              Inches.of(7.862498)), // values need to be changed probably
          new Rotation3d(
              Degrees.of(0),
              Degrees.of(-25),
              Degrees.of(200))); // values need to be changed probably

  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";
  public static String limelightCameraName = "limelight"; // limelight string name

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.07; // Meters
  public static double angularStdDevBaseline = 0.3; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static enum Camera {
    FrontApriltag(
        "Front OV9782",
        0.6,
        5,
        new Transform3d(
            new Translation3d(Inches.of(+13.6), Inches.of(0.25), Inches.of(+20.875)),
            new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(0)))),
    BackApriltag(
        "Back OV9281",
        1,
        4,
        new Transform3d(
            new Translation3d(Inches.of(-13.663), Inches.of(-4.75), Inches.of(19.625)),
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180)))),
    NoteVision(
        "Note Cam",
        0,
        0,
        new Transform3d(
            new Translation3d(Inches.of(-14.047), Inches.of(+0), Inches.of(+12.584)),
            new Rotation3d(0, Degrees.of(15).in(Radians), Math.PI)));

    public final String hardwareName;
    private final Transform3d intermediateToCamera;
    public final double cameraStdCoef;
    public final double trustDistance;
    private Supplier<Transform3d> robotToIntermediate;

    Camera(
        String hardwareName,
        double cameraStdCoef,
        double trustDistance,
        Transform3d finalToCamera) {
      this.hardwareName = hardwareName;
      this.cameraStdCoef = cameraStdCoef;
      this.trustDistance = trustDistance;
      this.intermediateToCamera = finalToCamera;
      this.robotToIntermediate = Transform3d::new;
    }

    @SuppressWarnings("unused")
    private static Transform3d robotToCameraFromCalibTag(
        Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
      return robotToCalibTag.plus(cameraToCalibTag.inverse());
    }

    public Camera withRobotToIntermediate(Supplier<Transform3d> robotToFinal) {
      this.robotToIntermediate = robotToFinal;
      return this;
    }

    public Transform3d getRobotToCam() {
      return robotToIntermediate.get().plus(intermediateToCamera);
    }
  }

  // TODO: figure out vision stdDevs
  public static final double singleTagAmbiguityCutoff = 0.05;
  public static final double minimumStdDev = 0.5;
  public static final double stdDevEulerMultiplier = 0.3;
  public static final double stdDevDistanceMultiplier = 0.4;
}
