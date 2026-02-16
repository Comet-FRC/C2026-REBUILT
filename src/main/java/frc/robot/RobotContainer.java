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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.MANUAL_CONTROL_VOLTAGE;
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
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOReal;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.*;
import frc.robot.subsystems.turret.*;
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
  private final Turret turret;
  private final Hood hood;
  private SwerveDriveSimulation driveSimulation = null;

  // Auto-aim command (stored as field so AutoFireCommand can access shot parameters)
  private AutoAimCommand autoAimCommand;

  // Controller
  private final CometLogitechController controller = new CometLogitechController(0);

  // Tunable values
  private final LoggedTunableNumber intakeWheelVolts =
      new LoggedTunableNumber("Intake/WheelVolts", 5.0);

  private final LoggedTunableNumber intakeAngle = new LoggedTunableNumber("Intake/Angle", 179.0);

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
        this.turret = new Turret(new TurretIOReal());
        this.hood = new Hood(new HoodIOReal());
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
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
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
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
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

    this.intake.setDefaultCommand(
        Commands.run(
            () -> {
              this.intake.setIntakeState(
                  Degrees.of(intakeAngle.get()), Volts.of(intakeWheelVolts.get()));
            },
            this.intake));

    this.indexer.setDefaultCommand(
        this.indexer.setRollerVoltage(() -> Volts.of(indexerRollerVolts.get())));

    this.kicker.setDefaultCommand(this.kicker.setVoltage(() -> Volts.of(0.0)));

    // Auto-aim: turret continuously tracks target, flywheel spins up to calculated speed
    this.autoAimCommand = new AutoAimCommand(drive, turret, flywheel, hood);

    this.turret.setDefaultCommand(autoAimCommand);
    this.flywheel.setDefaultCommand(autoAimCommand);
    this.hood.setDefaultCommand(autoAimCommand);
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
    controller.x().whileTrue(this.intake.setWheelVoltage(() -> Volts.of(-9)));

    // reset Gyro
    controller
        .a()
        .onTrue(
            Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive).ignoringDisable(true));

    controller.y().whileTrue(this.intake.setPivotVoltage(() -> Volts.of(2)));

    controller.b().whileTrue(this.intake.setPivotVoltage(() -> Volts.of(-2)));

    // Manual turret control
    controller
        .leftBumper()
        .whileTrue(this.turret.setVoltage(() -> Volts.of(MANUAL_CONTROL_VOLTAGE)));
    controller
        .rightBumper()
        .whileTrue(this.turret.setVoltage(() -> Volts.of(-MANUAL_CONTROL_VOLTAGE)));

    // TODO: CHECK LOGIC
    // // Auto-fire: when held, kicker fires automatically when turret is aimed + flywheel at speed
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         new AutoFireCommand(turret, flywheel, kicker, autoAimCommand::getLatestParameters));
    
    // Simple Shoot Button (Right Trigger)
    // Overrides turret tracking (which stops due to requirement conflict)
    controller
        .rightTrigger()
        .whileTrue(
            flywheel
                .setWheelVelocity(() -> RPM.of(3000))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.atSpeed(RPM.of(3000), RPM.of(100)))
                        .andThen(
                            Commands.parallel(
                                kicker.setVoltage(() -> Volts.of(4)),
                                indexer.setRollerVoltage(() -> Volts.of(4))))));

    if (Constants.currentMode == Constants.Mode.SIM) {
      configureSimulationBindings();
    }
  }

  private void configureSimulationBindings() {
    // Test Q1 (Top Left)
    controller
        .up()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(0))), drive));

    // Test Q2 (Top Right)
    controller
        .right()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(14.0, 6.0, Rotation2d.fromDegrees(180))), drive));

    // Test Q4 (Bottom Right)
    controller
        .down()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(14.0, 2.0, Rotation2d.fromDegrees(180))), drive));

    // Test Q3 (Bottom Left)
    controller
        .left()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0))), drive));
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

    // AdvantageScope custom robot configuration
    // Set to true for calibration (zeroed poses), false for real animation
    final boolean CALIBRATION_MODE = false;

    if (CALIBRATION_MODE) {
      // CALIBRATION: Publish zeroed poses to align models at origin
      Logger.recordOutput("AdvantageScope/RobotPose", new Pose2d());
      Logger.recordOutput(
          "AdvantageScope/ComponentPoses",
          new Pose3d[] {
            new Pose3d(), // Component 0 (Intake) - at origin
            new Pose3d() // Component 1 (Shooter) - at origin
          });
    } else {
      // NORMAL: Publish real component poses for animation
      Logger.recordOutput(
          "AdvantageScope/ComponentPoses",
          new Pose3d[] {
            intake.getComponentPose(), // Component 0 (Intake) - animated
            new Pose3d() // Component 1 (Shooter) - static
          });
    }
  }
}
