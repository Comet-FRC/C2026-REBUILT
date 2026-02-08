// Copyright 2021-2025 FRC 6328
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

package frc.robot;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOReal;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.*;
// import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.controller.CometLogitechController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Kicker kicker;
  private final Flywheel flywheel;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CometLogitechController controller = new CometLogitechController(0);

  // Tunable values
  private final LoggedTunableNumber intakeWheelVolts =
      new LoggedTunableNumber("Intake/WheelVolts", 0.0);

  private final LoggedTunableNumber indexerRollerVolts =
      new LoggedTunableNumber("Indexer/RollerVolts", 0.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (robotPose) -> {});
        this.vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(Camera.FrontApriltag),
                new VisionIOPhotonVision(Camera.BackApriltag),
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
        this.intake = new Intake(new IntakeIOReal());
        this.indexer = new Indexer(new IndexerIOReal());
        this.kicker = new Kicker(new KickerIOReal());
        this.flywheel = new Flywheel(new FlywheelIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose),
                // limelight camera added as photonvision sim
                new VisionIOPhotonVisionSim(
                    limelightCameraName,
                    robotTolimelightCamera,
                    driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        kicker = new Kicker(new KickerIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        kicker = new Kicker(new KickerIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    setupDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void setupDefaultCommands() {
    this.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // this.intake.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           this.intake.setIntakeState(
    //               IntakeConstants.INTAKING_ANGLE, Volts.of(intakeWheelVolts.get()));
    //         },
    //         this.intake));

    this.indexer.setDefaultCommand(
        this.indexer.setRollerVoltage(() -> Volts.of(indexerRollerVolts.get())));

    this.kicker.setDefaultCommand(this.kicker.setVoltage(() -> Volts.of(0.0)));

    // attempt at setDefaultCommand for flywheel
    this.flywheel.setDefaultCommand(this.flywheel.setWheelVoltage(() -> Volts.of(0.0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // reset Gyro
    controller
        .a()
        .onTrue(
            Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive).ignoringDisable(true));

    controller
        .y()
        .toggleOnTrue(
            Commands.parallel(
                this.intake.setWheelVoltage(() -> Volts.of(5.0)),
                this.indexer.setRollerVoltage(() -> Volts.of(5.0)),
                this.kicker.setVoltage(() -> Volts.of(5.0))));
    // outtake
    controller
        .b()
        .whileTrue(
            Commands.parallel(
                this.intake.setWheelVoltage(() -> Volts.of(-5.0)),
                this.indexer.setRollerVoltage(() -> Volts.of(-5.0)),
                this.kicker.setVoltage(() -> Volts.of(-5.0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(2, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

    // AdvantageScope custom robot calibration poses
    // Use these for calibration: set robot pose to "ZeroedRobotPose" and components to
    // "ZeroedComponentPoses"
    // Once calibrated, switch to actual mechanism poses
    Logger.recordOutput("AdvantageScope/ZeroedRobotPose", new Pose2d());
    Logger.recordOutput(
        "AdvantageScope/ZeroedComponentPoses",
        new Pose3d[] {
          new Pose3d(), // Component 0 (Intake) - zeroed at origin
          new Pose3d() // Component 1 (Shooter) - zeroed at origin
        });
  }
}
