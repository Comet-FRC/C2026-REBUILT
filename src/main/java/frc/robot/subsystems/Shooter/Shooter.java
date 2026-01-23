// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Shooter;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// public class Shooter extends SubsystemBase {

//   private final TalonFX shooter;
//   private final VelocityVoltage control = new VelocityVoltage(0).withEnableFOC(true);

//   //double motorSpeed;

//   public Shooter() {
//     shooter = new TalonFX(0);
//     this.setupMotor();
//   }

//   private void setupMotor(){
//     var shooterMotorConfig = new TalonFXConfiguration();
//     shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//     shooterMotorConfig.Voltage.PeakForwardVoltage = 0; //needs to be a constant
//     shooterMotorConfig.Voltage.PeakReverseVoltage = 0; //needs to be a constant

//     shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//     shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 0; //needs to be a value
//     shooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0; //needs to be a value, not sure
// if SupplyCurrentLowerTime is correct

//     //defines PID and other values

//     shooter.getConfigurator().apply(shooterMotorConfig);
//   }

//   private double toRPM(double rps){
//     return rps * 60.0;
//   }

//   private double toRPS(double rpm){
//     return rpm / 60.0;
//   }

//   /* public Command setVelocityFromDistance(DoubleSupplier distance) {
// 		return
// 			Commands.runOnce(
// 				() -> {
// 					RobotContainer.setState(State.REVVING);
// 				},
// 				this
// 			).andThen(this.setVelocity(() -> RANGE_TABLE.get(distance.getAsDouble())));
//   } */

//   public Command setVelocity(Supplier<ShooterSpeed> shooterSpeed){
//     return
//       Commands.runOnce(() -> {
//         double motorSpeed = shooterSpeed.get().motorSpeed;
//         this.shooter.setControl(this.control.withVelocity(this.toRPS(motorSpeed)));
//       }, this);
//   }

//   public Command shoot(){
//     return
//       this.setVelocity(
//         () -> {
//           double shooterSpeed = SmartDashboard.getNumber("shooter/motorSpeed", 0);
//           return new ShooterSpeed(shooterSpeed);
//         }
//       )
//       .andThen(
//         new WaitUntilCommand(() -> !ProximitySensor.getInstance().hasObject()) //somehow import
// proximity sensor
//       );
//   }

//   public boolean isReady(){
//     boolean isReady = Math.abs(toRPM(shooter.getClosedLoopError().getValueAsDouble())) < 0;
// //needs value
//     return isReady;
//   }

//   public Command eject() {
//     return this.setVelocity(() -> new ShooterSpeed(0)); // need values
//   }

//   public Command stop(){
//     return this.setVelocity(() -> new ShooterSpeed(0));
//   }

//   @Override
//   public void periodic() {
//     double targetMotorSpeed = toRPM(shooter.getClosedLoopReference().getValueAsDouble());
//     double measuredMotorSpeed = toRPM(shooter.getVelocity().getValueAsDouble());
//     double speedError = toRPM(shooter.getClosedLoopError().getValueAsDouble());

//     //logging stuff
//   }
// }
