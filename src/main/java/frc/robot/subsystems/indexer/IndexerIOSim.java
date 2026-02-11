// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

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

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim rollerMotor = configureRollerMotor();

  private static DCMotorSim configureRollerMotor() {
    DCMotor rollerGearbox = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> wheelPlant =
        LinearSystemId.createDCMotorSystem(
            rollerGearbox,
            IndexerConstants.WHEEL_MOMENT_OF_INERTIA,
            IndexerConstants.WHEEL_CONVERSION_FACTOR);
    return new DCMotorSim(wheelPlant, rollerGearbox);
  }

  private final PIDController rollerPID =
      new PIDController(
          IndexerConstants.ROLLER_SIM_kP,
          IndexerConstants.ROLLER_SIM_kI,
          IndexerConstants.ROLLER_SIM_kD);

  private boolean rollerVoltageMode = false;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    rollerMotor.update(0.02);

    runLoopControl();

    inputs.rollerPosition = rollerMotor.getAngularPosition();
    inputs.rollerVelocity = rollerMotor.getAngularVelocity();
    inputs.rollerDesiredVelocity = RadiansPerSecond.of(rollerPID.getSetpoint());
    inputs.rollerAppliedVolts = Volts.of(rollerMotor.getInputVoltage());
    inputs.rollerSupplyCurrent = Amps.of(rollerMotor.getCurrentDrawAmps());
  }

  private void runLoopControl() {
    if (!rollerVoltageMode) {

      rollerMotor.setInputVoltage(
          rollerPID.calculate(rollerMotor.getAngularVelocity().in(RadiansPerSecond))
          // +
          // rollerFF.calculate(rollerPID.getSetpoint())
          );
    }
  }

  @Override
  public void setRollerVelocitySetpoint(AngularVelocity rollerVelocity) {
    this.rollerVoltageMode = false;
    this.rollerPID.setSetpoint(rollerVelocity.in(RadiansPerSecond));
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    this.rollerVoltageMode = true;
    this.rollerMotor.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void stopRoller() {
    this.setRollerVoltage(Volts.of(0.0));
  }
}
