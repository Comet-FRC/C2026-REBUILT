// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KickerIOSim implements KickerIO {
  private final DCMotorSim leftMotor = configureMotor();
  private final DCMotorSim rightMotor = configureMotor();

  private static DCMotorSim configureMotor() {
    DCMotor gearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> wheelPlant =
        LinearSystemId.createDCMotorSystem(
            gearbox,
            KickerConstants.WHEEL_MOMENT_OF_INERTIA,
            KickerConstants.WHEEL_CONVERSION_FACTOR);
    return new DCMotorSim(wheelPlant, gearbox);
  }

  private final PIDController leftPID =
      new PIDController(
          KickerConstants.ROLLER_SIM_kP,
          KickerConstants.ROLLER_SIM_kI,
          KickerConstants.ROLLER_SIM_kD);

  private final PIDController rightPID =
      new PIDController(
          KickerConstants.ROLLER_SIM_kP,
          KickerConstants.ROLLER_SIM_kI,
          KickerConstants.ROLLER_SIM_kD);

  private boolean voltageMode = false;

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    leftMotor.update(0.02);
    rightMotor.update(0.02);

    runLoopControl();

    // Left motor inputs
    inputs.leftPosition = leftMotor.getAngularPosition();
    inputs.leftVelocity = leftMotor.getAngularVelocity();
    inputs.leftDesiredVelocity = RadiansPerSecond.of(leftPID.getSetpoint());
    inputs.leftAppliedVolts = Volts.of(leftMotor.getInputVoltage());
    inputs.leftSupplyCurrent = Amps.of(leftMotor.getCurrentDrawAmps());

    // Right motor inputs
    inputs.rightPosition = rightMotor.getAngularPosition();
    inputs.rightVelocity = rightMotor.getAngularVelocity();
    inputs.rightDesiredVelocity = RadiansPerSecond.of(rightPID.getSetpoint());
    inputs.rightAppliedVolts = Volts.of(rightMotor.getInputVoltage());
    inputs.rightSupplyCurrent = Amps.of(rightMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!voltageMode) {
      leftMotor.setInputVoltage(
          leftPID.calculate(leftMotor.getAngularVelocity().in(RadiansPerSecond)));
      rightMotor.setInputVoltage(
          rightPID.calculate(rightMotor.getAngularVelocity().in(RadiansPerSecond)));
    }
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity velocity) {
    this.voltageMode = false;
    this.leftPID.setSetpoint(velocity.in(RadiansPerSecond));
    this.rightPID.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    this.leftMotor.setInputVoltage(volts.in(Volts));
    this.rightMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stop() {
    this.setVoltage(Volts.of(0.0));
  }
}
