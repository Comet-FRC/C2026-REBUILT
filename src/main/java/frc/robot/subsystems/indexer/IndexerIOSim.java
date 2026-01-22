// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim wheelMotor = configurewheelMotor();

  private static DCMotorSim configurewheelMotor() {
    DCMotor wheelGearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> wheelPlant =
        LinearSystemId.createDCMotorSystem(
            wheelGearbox, IndexerConstants.WHEEL_MOI, IndexerConstants.WHEEL_CONVERSION_FACTOR);
    return new DCMotorSim(wheelPlant, wheelGearbox);
  }

  private final PIDController pid =
      new PIDController(IndexerConstants.SIM_kP, IndexerConstants.SIM_kI, IndexerConstants.SIM_kD);

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(
          IndexerConstants.SIM_kS, IndexerConstants.SIM_kV, IndexerConstants.SIM_kA);

  private boolean voltageMode = false;
  private final MutVoltage appliedVoltage = Volts.mutable(0);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    wheelMotor.update(0.02);

    runLoopControl();

    inputs.WheelPosition = wheelMotor.getAngularPosition();
    inputs.WheelVelocity = wheelMotor.getAngularVelocity();
    inputs.WheelDesiredVelocity = RadiansPerSecond.of(pid.getSetpoint());
    inputs.WheelAppliedVoltage = Volts.of(wheelMotor.getInputVoltage());
    inputs.WheelSupplyCurrent = Amps.of(wheelMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!voltageMode) {

      wheelMotor.setInputVoltage(
          pid.calculate(wheelMotor.getAngularVelocity().in(RadiansPerSecond))
          // +
          // wheelFF.calculate(topPID.getSetpoint())
          );
    }
  }

  @Override
  public void setWheelVelocitySetpoint(AngularVelocity Velocity) {
    this.voltageMode = false;
    this.pid.setSetpoint(Velocity.in(RadiansPerSecond));
  }

  @Override
  public void setVoltage(Voltage volts) {
    this.voltageMode = true;
    this.wheelMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stop() {
    this.setVoltage(Volts.of(0));
  }
}
