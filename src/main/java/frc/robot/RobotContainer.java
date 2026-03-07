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
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.TargetMode;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoFireCommand;
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
// import frc.robot.util.ProximitySensor;
import frc.robot.util.controller.*;
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

  // Target mode — toggle between feeding and hub targets (default: FEEDING)
  private TargetMode targetMode = TargetMode.FEEDING;

  // Auto-aim command (stored as field so AutoFireCommand can access shot parameters)
  private AutoAimCommand autoAimCommand;

  // Controller
  private final CometXboxController driverController = new CometXboxController(0);
  private final CometLogitechController operatorController = new CometLogitechController(1);

  private final LoggedTunableNumber intakeWheelVolts =
      new LoggedTunableNumber("Intake/WheelVolts", 9);
  private final LoggedTunableNumber FlywheelVelocity =
      new LoggedTunableNumber("Flywheel/RPM", 3000.0);
  private final LoggedTunableNumber HoodAngle = new LoggedTunableNumber("Hood/Angle", 0.0);
  private final LoggedTunableNumber intakeAngle = new LoggedTunableNumber("Intake/Angle", 166.0);
  private final LoggedTunableNumber indexerRollerVolts =
      new LoggedTunableNumber("Indexer/RollerVolts", 2.0);
  private final LoggedTunableNumber turretVolts = new LoggedTunableNumber("Turret/Volts", 2.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // this.sensor = new ProximitySensor();
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
                drive::getPose,
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(Camera.LeftCamera),
                new VisionIOPhotonVision(Camera.RightCamera),
                new VisionIOLimelight("limelight", drive::getRotation));
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
                driveSimulation::getSimulatedDriveTrainPose,
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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        vision =
            new Vision(
                drive::getPose,
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        kicker = new Kicker(new KickerIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        break;
    }

    // ── Register PathPlanner named commands ──────────────────────────────────
    // IMPORTANT: Must be registered BEFORE AutoBuilder.buildAutoChooser()

    // NOTE ON TELEOP SAFETY: WPILib automatically cancels ALL running commands
    // when switching from auton → teleop. AutoAim will stop on its own. The
    // // driver's Y-button toggle re-enables it independently in teleop.
    NamedCommands.registerCommand(
        "Deploy Intake",
        Commands.runOnce(
            () ->
                intake.setIntakeState(
                    Degrees.of(intakeAngle.get()), Volts.of(intakeWheelVolts.get())),
            intake));

    // NamedCommands.registerCommand(
    //     "spinUp",
    //     Commands.run(
    //         () -> flywheel.io.setWheelVelocitySetpoint(RPM.of(FlywheelVelocity.get())),
    // flywheel));

    // // Each named command creates one AutoAimCommand instance that is shared between:
    // //   1. The Commands.parallel() group (so it runs and owns the turret), and
    // //   2. AutoFireCommand (so it reads latestParameters from the SAME object).
    // // Using two separate instances would cause them to fight over the turret requirement.
    AutoAimCommand shootAim = new AutoAimCommand(drive, turret, () -> TargetMode.HUB);
    NamedCommands.registerCommand(
        "Shoot On Move",
        Commands.parallel(
            shootAim, new AutoFireCommand(shootAim, turret, flywheel, hood, kicker, indexer)));

    // AutoAimCommand shootTimedAim = new AutoAimCommand(drive, turret, () -> TargetMode.HUB);
    // NamedCommands.registerCommand(
    //     "shootTimed",
    //     Commands.parallel(
    //             shootTimedAim,
    //             new AutoFireCommand(shootTimedAim, turret, flywheel, hood, kicker, indexer))
    //         .withTimeout(3.0));

    AutoAimCommand shootFeedAim = new AutoAimCommand(drive, turret, () -> TargetMode.FEEDING);
    NamedCommands.registerCommand(
        "Feeding",
        Commands.parallel(
            shootFeedAim,
            new AutoFireCommand(shootFeedAim, turret, flywheel, hood, kicker, indexer)));

    // AutoAimCommand shootFeedTimedAim = new AutoAimCommand(drive, turret, () ->
    // TargetMode.FEEDING);
    // NamedCommands.registerCommand(
    //     "shootFeedTimed",
    //     Commands.parallel(
    //             shootFeedTimedAim,
    //             new AutoFireCommand(shootFeedTimedAim, turret, flywheel, hood, kicker, indexer))
    //         .withTimeout(3.0));

    // // NOTE: autoAim/autoAimFeed never finish on their own — always put them
    // // inside a Race/Deadline Group alongside a path so the path acts as the deadline.
    // NamedCommands.registerCommand(
    //     "autoAim", new AutoAimCommand(drive, turret, () -> TargetMode.HUB));

    // NamedCommands.registerCommand(
    //     "autoAimFeed", new AutoAimCommand(drive, turret, () -> TargetMode.FEEDING));

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    setupDefaultCommands();
    configureButtonBindings();
  }

  private void setupDefaultCommands() {
    this.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    this.intake.setDefaultCommand(
        Commands.run(
            () -> {
              this.intake.setIntakeState(Degrees.of(80.0), Volts.of(0.0));
            },
            this.intake));

    // this.intake.setDefaultCommand(this.intake.setPivotVoltage(() -> Volts.of(0.0)));

    this.indexer.setDefaultCommand(
        this.indexer.setRollerVoltage(() -> Volts.of(indexerRollerVolts.get())));
    this.kicker.setDefaultCommand(this.kicker.setVoltage(() -> Volts.of(0.0)));
    this.flywheel.setDefaultCommand(this.flywheel.setWheelVoltage(() -> Volts.of(0.0)));
    this.hood.setDefaultCommand(this.hood.setPosition(() -> Degrees.of(0.0)));
    this.turret.setDefaultCommand(this.turret.setVoltage(() -> Volts.of(0.0)));

    this.autoAimCommand = new AutoAimCommand(drive, turret, () -> targetMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // MENU + MENU = RESET GYRO
    driverController
        .leftMenu()
        .and(driverController.rightMenu())
        .onTrue(
            Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive).ignoringDisable(true));

    // RB = Toggle target mode: FEEDING ↔ HUB
    driverController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      targetMode =
                          (targetMode == TargetMode.FEEDING) ? TargetMode.HUB : TargetMode.FEEDING;
                      Logger.recordOutput("AutoAim/TargetMode", targetMode.name());
                    })
                .ignoringDisable(true));

    // LB = Manual Kicker
    driverController.leftBumper().whileTrue(this.kicker.setVoltage(() -> Volts.of(4.0)));

    // LT = Intake while true move to intaking angle and run volts
    driverController
        .leftTrigger()
        .toggleOnTrue(
            Commands.run(
                () ->
                    this.intake.setIntakeState(
                        Degrees.of(intakeAngle.get()), Volts.of(intakeWheelVolts.get())),
                this.intake));

    // Y = Toggle AutoAim
    driverController.y().toggleOnTrue(this.autoAimCommand);

    // RT = AutoFire: spin up flywheel, set hood, and fire kicker
    // AutoAimCommand (Y-toggle) must be active concurrently — it owns the turret.
    driverController
        .rightTrigger()
        .whileTrue(new AutoFireCommand(autoAimCommand, turret, flywheel, hood, kicker, indexer));

    driverController.left().onTrue(this.hood.setPosition(() -> Degrees.of(HoodAngle.get())));

    // Manual FEEDING target override (Right DPAD)
    driverController
        .right()
        .onTrue(
            Commands.runOnce(() -> FieldConstants.toggleManualFeedingOverride())
                .ignoringDisable(true));

    // Turret Manual Control
    operatorController
        .leftBumper()
        .whileTrue(this.turret.setVoltage(() -> Volts.of(-turretVolts.get())));
    operatorController
        .rightBumper()
        .whileTrue(this.turret.setVoltage(() -> Volts.of(turretVolts.get())));

    // Hood Manual Control
    operatorController.up().whileTrue(this.hood.setVoltage(() -> Volts.of(3)));
    operatorController.down().whileTrue(this.hood.setVoltage(() -> Volts.of(-3)));

    operatorController.right().whileTrue(this.kicker.setVoltage(() -> Volts.of(5)));
    // Flywheel RPM Control (spin up only, no fire)
    operatorController
        .rightTrigger()
        .whileTrue(this.flywheel.setWheelVelocity(() -> RPM.of(FlywheelVelocity.get())));

    // Manual Shoot: spin flywheel + set hood, auto-fire when within 70 RPM
    driverController
        .down()
        .whileTrue(
            Commands.run(
                () -> {
                  var targetRPM = RPM.of(FlywheelVelocity.get());
                  var targetHood = Degrees.of(HoodAngle.get());
                  flywheel.io.setWheelVelocitySetpoint(targetRPM);
                  hood.io.setPositionSetpoint(targetHood);
                  boolean flywheelReady = flywheel.atSpeed(targetRPM, RPM.of(70.0));
                  if (flywheelReady) {
                    kicker.io.setVoltage(Volts.of(4.0));
                    indexer.io.setRollerVoltage(Volts.of(4.0));
                  } else {
                    kicker.io.setVoltage(Volts.of(0.0));
                    indexer.io.setRollerVoltage(Volts.of(0.0));
                  }
                },
                flywheel,
                hood,
                kicker,
                indexer))
        .onFalse(
            Commands.runOnce(
                () -> {
                  flywheel.io.setWheelVelocitySetpoint(RPM.of(0));
                  hood.io.stop();
                  kicker.io.setVoltage(Volts.of(0.0));
                  indexer.io.setRollerVoltage(Volts.of(0.0));
                },
                flywheel,
                hood,
                kicker,
                indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    Logger.recordOutput("Dashboard/MatchTimeSec", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/TargetMode", targetMode.name());

    double fieldRelativeDeg =
        (turret.getAngle().in(Degrees) + drive.getRotation().getDegrees()) % 360.0;
    if (fieldRelativeDeg < 0) fieldRelativeDeg += 360.0;
    Logger.recordOutput("Dashboard/TurretFieldRelativeAngleDeg", fieldRelativeDeg);
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

    Pose3d simulatedRobotPose3d = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Camera0Pose",
        simulatedRobotPose3d.transformBy(VisionConstants.robotToCamera0));
    Logger.recordOutput(
        "FieldSimulation/Camera1Pose",
        simulatedRobotPose3d.transformBy(VisionConstants.robotToCamera1));
    Logger.recordOutput(
        "FieldSimulation/LimelightPose",
        simulatedRobotPose3d.transformBy(VisionConstants.robotTolimelightCamera));

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
