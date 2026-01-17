// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter.ShooterSpeed;

public class Intake extends SubsystemBase {

  private final TalonFX intake;
  private final VelocityVoltage control = new VelocityVoltage(0).withEnableFOC(true);

  public Intake() {
    intake = new TalonFX(0);
    this.setupMotor();
  }

  private void setupMotor(){
    var intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotorConfig.Voltage.PeakForwardVoltage = 0; //needs to be a constant
    intakeMotorConfig.Voltage.PeakReverseVoltage = 0; //needs to be constant

    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 0; //needs to be value
    intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0; //needs to be value, not sure if SupplyCurrentLowerTime is correct

    //defines PID and other values

    intake.getConfigurator().apply(intakeMotorConfig);
  }

  private double toRPM(double rps){
    return rps * 60.0;
  }

  private double toRPS(double rpm){
    return rpm / 60.0;
  }

  /* public Command setVelocityFromDistance(DoubleSupplier distance) {
		return 
			Commands.runOnce(
				() -> {
					RobotContainer.setState(State.REVVING);
				},
				this
			).andThen(this.setVelocity(() -> RANGE_TABLE.get(distance.getAsDouble())));
  } */

  public Command setVelocity(Supplier<IntakeSpeed> intakeSpeed){
    return
      Commands.runOnce(() -> {
        double motorSpeed = intakeSpeed.get().motorSpeed;
        this.intake.setControl(this.control.withVelocity(this.toRPS(motorSpeed)));
      }, this);
  }

  public Command intake(){
    return
      this.setVelocity(
        () -> {
          double intakeSpeed = SmartDashboard.getNumber("intake/motorSpeed", 0);
          return new IntakeSpeed(intakeSpeed);
        }
      )
      .andThen(
        new WaitUntilCommand(() -> !ProximitySensor.getInstance().hasObject()) //somehow import proximity sensor
      );
  }

  public boolean isReady(){
    boolean isReady = Math.abs(toRPM(intake.getClosedLoopError().getValueAsDouble())) < 0; //needs value
    return isReady;
  }

  public Command intakeIn() {
    return this.setVelocity(() -> new IntakeSpeed(0)); // need values
  }

  public Command stop(){
    return this.setVelocity(() -> new IntakeSpeed(0));
  }

  @Override
  public void periodic() {
    double targetMotorSpeed = toRPM(intake.getClosedLoopReference().getValueAsDouble());
    double measuredMotorSpeed = toRPM(intake.getVelocity().getValueAsDouble());
    double speedError = toRPM(intake.getClosedLoopError().getValueAsDouble());

    //logging stuff
  }
}
