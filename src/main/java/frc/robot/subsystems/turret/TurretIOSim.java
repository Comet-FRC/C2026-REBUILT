package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim;
  private final PIDController controller;

  private Rotation2d targetPosition = TurretConstants.TURRET_STARTING_OFFSET;
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  public TurretIOSim() {
    // Create a single joint simulation for the turret
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), TurretConstants.SIM_MOI, TurretConstants.GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    // Simple PID controller for simulation
    controller = new PIDController(10.0, 0.0, 0.5);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    // Set initial position
    sim.setState(TurretConstants.TURRET_STARTING_OFFSET.getRadians(), 0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control if enabled
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              controller.calculate(sim.getAngularPositionRad(), targetPosition.getRadians()),
              -12.0,
              12.0);
    }

    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.position = Radians.of(sim.getAngularPositionRad());
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.statorCurrent = Amps.of(sim.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(sim.getCurrentDrawAmps());
    inputs.targetPosition = Radians.of(targetPosition.getRadians());
  }

  @Override
  public void setPosition(Rotation2d position) {
    closedLoop = true;
    targetPosition = position;
  }

  @Override
  public void setVoltage(Voltage volts) {
    closedLoop = false;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void resetPosition(Rotation2d position) {
    sim.setState(position.getRadians(), sim.getAngularVelocityRadPerSec());
  }
}
