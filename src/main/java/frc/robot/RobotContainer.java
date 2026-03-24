package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.TargetMode;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoFireCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotParameters;
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
import frc.robot.util.controller.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  // AutoAim state
  private boolean autoAimEnabled = true;
  private int manualTargetOverride = -1; // -1 = AUTO, 0 = FORCE_HUB, 1 = FORCE_FEEDING
  private AutoAimCommand autoAimCommand;

  // Controllers
  private final CometXboxController driverController = new CometXboxController(0);
  private final CometLogitechController operatorController = new CometLogitechController(1);

  // Tunable numbers
  private final LoggedTunableNumber intakeWheelVolts =
      new LoggedTunableNumber("Intake/WheelVolts", 8);
  private final LoggedTunableNumber FlywheelVelocity =
      new LoggedTunableNumber("Flywheel/RPM", 600.0);
  private final LoggedTunableNumber HoodAngle = new LoggedTunableNumber("Hood/Angle", 0.0);
  private final LoggedTunableNumber intakeAngle = new LoggedTunableNumber("Intake/Angle", 11.0);
  private final LoggedTunableNumber indexerRollerVolts =
      new LoggedTunableNumber("Indexer/RollerVolts", 5.0);
  private final LoggedTunableNumber turretVolts = new LoggedTunableNumber("Turret/Volts", 2.0);

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

    // Named commands (must register BEFORE AutoBuilder.buildAutoChooser())
    NamedCommands.registerCommand(
        "Deploy Intake",
        intake.runIntakeDelayed(
            () -> Degrees.of(intakeAngle.get()), () -> Volts.of(intakeWheelVolts.get())));

    // NamedCommands.registerCommand(
    //     "spinUp",
    //     Commands.run(
    //         () -> flywheel.io.setWheelVelocitySetpoint(RPM.of(FlywheelVelocity.get())),
    // flywheel));

    AutoAimCommand shootAim = new AutoAimCommand(drive, turret, () -> TargetMode.HUB);
    NamedCommands.registerCommand(
        "Shoot On Move",
        Commands.parallel(
            shootAim,
            new AutoFireCommand(
                shootAim::getLatestParameters, turret, flywheel, hood, kicker, indexer)));

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
            new AutoFireCommand(
                shootFeedAim::getLatestParameters, turret, flywheel, hood, kicker, indexer)));

    // AutoAimCommand shootFeedTimedAim = new AutoAimCommand(drive, turret, () ->
    // TargetMode.FEEDING);
    // NamedCommands.registerCommand(
    //     "shootFeedTimed",
    //     Commands.parallel(
    //             shootFeedTimedAim,
    //             new AutoFireCommand(shootFeedTimedAim, turret, flywheel, hood, kicker, indexer))
    //         .withTimeout(3.0));

    // NamedCommands.registerCommand(
    //     "autoAim", new AutoAimCommand(drive, turret, () -> TargetMode.HUB));

    // NamedCommands.registerCommand(
    //     "autoAimFeed", new AutoAimCommand(drive, turret, () -> TargetMode.FEEDING));

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());

    AutoAimCommand shootHubAim = new AutoAimCommand(drive, turret, () -> TargetMode.HUB);
    autoChooser.addOption(
        "Shoot On Hub",
        Commands.parallel(
                shootHubAim,
                new AutoFireCommand(
                    shootHubAim::getLatestParameters, turret, flywheel, hood, kicker, indexer))
            .withTimeout(15.0));

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
              this.intake.setIntakeState(Degrees.of(85.0), Volts.of(0.0));
            },
            this.intake));

    // this.intake.setDefaultCommand(this.intake.setPivotVoltage(() -> Volts.of(0.0)));

    this.indexer.setDefaultCommand(
        this.indexer.setRollerVoltage(() -> Volts.of(indexerRollerVolts.get())));
    this.kicker.setDefaultCommand(this.kicker.setVoltage(() -> Volts.of(0.0)));
    this.flywheel.setDefaultCommand(this.flywheel.setWheelVoltage(() -> Volts.of(0.0)));
    this.hood.setDefaultCommand(this.hood.setPosition(() -> Degrees.of(0.0)));

    // AutoAim is the turret's default command — always running
    this.autoAimCommand = new AutoAimCommand(drive, turret, this::resolveTargetMode);
    this.turret.setDefaultCommand(this.autoAimCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ── Driver ────────────────────────────────────────────────────────────────
    driverController
        .leftMenu()
        .and(driverController.rightMenu())
        .onTrue(
            Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive).ignoringDisable(true));

    driverController.a().whileTrue(this.kicker.setVoltage(() -> Volts.of(4.0)));

    driverController
        .rightBumper()
        .whileTrue(
            this.intake.runIntakeDelayed(
                () -> Degrees.of(intakeAngle.get()), () -> Volts.of(intakeWheelVolts.get())));

    driverController
        .rightTrigger()
        .whileTrue(
            new AutoFireCommand(
                autoAimCommand::getLatestParameters, turret, flywheel, hood, kicker, indexer));

    driverController.left().onTrue(this.hood.setPosition(() -> Degrees.of(HoodAngle.get())));
    driverController
        .right()
        .onTrue(this.flywheel.setWheelVelocity(() -> RPM.of(FlywheelVelocity.get())));

    // driverController
    //     .right()
    //     .onTrue(
    //         Commands.runOnce(() -> FieldConstants.toggleManualFeedingOverride())
    //             .ignoringDisable(true));

    driverController
        .down()
        .whileTrue(
            Commands.run(
                () -> {
                  ShotParameters params =
                      ShotCalculator.calculate(
                          drive.getPose(),
                          drive.getFieldVelocity(),
                          turret.getAngle(),
                          resolveTargetMode());

                  var targetRPM = RPM.of(params.flywheelSpeedRPM());
                  var targetHood = Degrees.of(params.hoodAngleDegrees());
                  flywheel.io.setWheelVelocitySetpoint(targetRPM);
                  hood.io.setPositionSetpoint(targetHood);

                  if (flywheel.atSpeed(targetRPM, RPM.of(70.0))) {
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

    // ── Operator ──────────────────────────────────────────────────────────────
    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      autoAimEnabled = !autoAimEnabled;
                      Logger.recordOutput("AutoAim/Enabled", autoAimEnabled);
                      if (!autoAimEnabled) {
                        CommandScheduler.getInstance()
                            .schedule(turret.setPosition(() -> Degrees.of(180.0)));
                      }
                    })
                .ignoringDisable(true));

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      manualTargetOverride++;
                      if (manualTargetOverride > 1) manualTargetOverride = -1;
                      String label =
                          switch (manualTargetOverride) {
                            case 0 -> "FORCE_HUB";
                            case 1 -> "FORCE_FEEDING";
                            default -> "AUTO";
                          };
                      Logger.recordOutput("AutoAim/ManualOverride", label);
                    })
                .ignoringDisable(true));

    operatorController
        .leftBumper()
        .whileTrue(turret.setVoltage(() -> Volts.of(-turretVolts.get())));
    operatorController
        .rightBumper()
        .whileTrue(turret.setVoltage(() -> Volts.of(turretVolts.get())));

    operatorController.up().whileTrue(hood.setVoltage(() -> Volts.of(3)));
    operatorController.down().whileTrue(hood.setVoltage(() -> Volts.of(-3)));
    operatorController.right().whileTrue(kicker.setVoltage(() -> Volts.of(5)));
    operatorController
        .rightTrigger()
        .whileTrue(flywheel.setWheelVelocity(() -> RPM.of(FlywheelVelocity.get())));
  }

  private TargetMode resolveTargetMode() {
    if (manualTargetOverride == 0) return TargetMode.HUB;
    if (manualTargetOverride == 1) return TargetMode.FEEDING;
    // Auto: compute turret field-relative X from robot pose + turret offset
    Pose2d turretPose =
        drive
            .getPose()
            .transformBy(
                new Transform2d(
                    TurretConstants.PHYSICAL_OFFSET.unaryMinus(), Inches.zero(), Rotation2d.kZero));
    return FieldConstants.getAutoTargetMode(turretPose.getX());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    TargetMode currentMode = resolveTargetMode();
    Logger.recordOutput("Dashboard/MatchTimeSec", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/TargetMode", currentMode.name());
    Logger.recordOutput("Dashboard/AutoAimEnabled", autoAimEnabled);
    Logger.recordOutput(
        "Dashboard/ManualOverride",
        manualTargetOverride == -1
            ? "AUTO"
            : (manualTargetOverride == 0 ? "FORCE_HUB" : "FORCE_FEEDING"));

    double fieldRelativeDeg =
        (turret.getAngle().in(Degrees) + drive.getRotation().getDegrees()) % 360.0;
    if (fieldRelativeDeg < 0) fieldRelativeDeg += 360.0;
    Logger.recordOutput("Dashboard/TurretFieldRelativeAngleDeg", fieldRelativeDeg);

    // Publish to SmartDashboard for Elastic Dashboard widgets
    SmartDashboard.putString("Targeting Mode", currentMode.name());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Intake Angle (deg)", intake.getPivotPosition().in(Degrees));
    SmartDashboard.putBoolean("Auto Aim Enabled", autoAimEnabled);
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

    final boolean CALIBRATION_MODE = false;
    if (CALIBRATION_MODE) {
      Logger.recordOutput("AdvantageScope/RobotPose", new Pose2d());
      Logger.recordOutput(
          "AdvantageScope/ComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d()});
    } else {
      Logger.recordOutput(
          "AdvantageScope/ComponentPoses", new Pose3d[] {intake.getComponentPose(), new Pose3d()});
    }
  }
}
