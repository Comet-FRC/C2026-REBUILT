// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.subsystems.Kicker;

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
import java.util.function.Supplier;

public class Kicker extends SubsystemBase {

  private TalonFX kicker;
  private final VelocityVoltage control = new VelocityVoltage(0).withEnableFOC(true);

  public Kicker() {
    kicker = new TalonFX(0); // needs device id
    this.setupMotor();
  }

  private void setupMotor() {
    var intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotorConfig.Voltage.PeakForwardVoltage = 0; // needs to be a constant
    intakeMotorConfig.Voltage.PeakReverseVoltage = 0; // needs to be constant

    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 0; // needs to be value
    intakeMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
        0; // needs to be value, not sure if SupplyCurrentLowerTime is correct

    // defines PID and other values

    kicker.getConfigurator().apply(intakeMotorConfig);
  }

  private double toRPM(double rps) {
    return rps * 60.0;
  }

  private double toRPS(double rpm) {
    return rpm / 60.0;
  }
    */

  /* public Command setVelocityFromDistance(DoubleSupplier distance) {
  return
  	Commands.runOnce(
  		() -> {
  			RobotContainer.setState(State.REVVING);
  		},
  		this
  	).andThen(this.setVelocity(() -> RANGE_TABLE.get(distance.getAsDouble())));
  } */

  /*
    public Command setVelocity(Supplier<KickerSpeed> kickerSpeed) {
      return Commands.runOnce(
          () -> {
            double motorSpeed = kickerSpeed.get().motorSpeed;
            this.kicker.setControl(this.control.withVelocity(this.toRPS(motorSpeed)));
          },
          this);
    }

    public Command kicker() {
      return this.setVelocity(
              () -> {
                double kickerSpeed = SmartDashboard.getNumber("intake/motorSpeed", 0);
                return new KickerSpeed(kickerSpeed);
              })
          .andThen(
              new WaitUntilCommand(
                  () -> !ProximitySensor.getInstance().hasObject()) // somehow import proximity sensor
              );
    }

    public boolean isReady() {
      boolean isReady =
          Math.abs(toRPM(kicker.getClosedLoopError().getValueAsDouble())) < 0; // needs value
      return isReady;
    }

    public Command kick() {
      return this.setVelocity(() -> new KickerSpeed(0)); // need values
    }

    public Command stop() {
      return this.setVelocity(() -> new KickerSpeed(0));
    }

    @Override
    public void periodic() {
      double targetMotorSpeed = toRPM(kicker.getClosedLoopReference().getValueAsDouble());
      double measuredMotorSpeed = toRPM(kicker.getVelocity().getValueAsDouble());
      double speedError = toRPM(kicker.getClosedLoopError().getValueAsDouble());

      // logging stuff
    }
  }*/
