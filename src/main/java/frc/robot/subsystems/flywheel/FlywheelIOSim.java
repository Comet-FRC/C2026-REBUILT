// package frc.robot.subsystems.flywheel;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import frc.robot.util.LoggedTunableNumber;

// public class FlywheelIOSim implements FlywheelIO {
//   private final DCMotorSim wheelMotor = configureWheelMotor();

//   private static DCMotorSim configureWheelMotor() {
//     DCMotor wheelGearbox = DCMotor.getNEO(1);
//     LinearSystem<N2, N1, N2> wheelPlant =
//         LinearSystemId.createDCMotorSystem(
//             wheelGearbox, FlywheelConstants.WHEEL_MOI, (18.0 / 24.0));
//     return new DCMotorSim(wheelPlant, wheelGearbox);
//   }

//   private final LoggedTunableNumber wheelKp =
//       new LoggedTunableNumber("FlywheelSim/Wheel/kP", FlywheelConstants.WHEEL_SIM_kP);
//   private final LoggedTunableNumber wheelKi =
//       new LoggedTunableNumber("FlywheelSim/Wheel/kI", FlywheelConstants.WHEEL_SIM_kI);
//   private final LoggedTunableNumber wheelKd =
//       new LoggedTunableNumber("FlywheelSim/Wheel/kD", FlywheelConstants.WHEEL_SIM_kD);

//   private final PIDController wheelPID =
//       new PIDController(
//           FlywheelConstants.WHEEL_SIM_kP,
//           FlywheelConstants.WHEEL_SIM_kI,
//           FlywheelConstants.WHEEL_SIM_kD);

//   private final SimpleMotorFeedforward wheelFF =
//       new SimpleMotorFeedforward(
//           FlywheelConstants.WHEEL_SIM_kS,
//           FlywheelConstants.WHEEL_SIM_kV,
//           FlywheelConstants.WHEEL_SIM_kA);

//   private boolean wheelVoltageMode = false;

//   private void updateTunableValues() {
//     // Update wheel PID if changed
//     LoggedTunableNumber.ifChanged(
//         hashCode(),
//         (values) -> {
//           wheelPID.setPID(values[0], values[1], values[2]);
//         },
//         wheelKp,
//         wheelKi,
//         wheelKd);
//   }

//   @Override
//   public void updateInputs(FlywheelIOInputs inputs) {
//     updateTunableValues();

//     wheelMotor.update(0.02);

//     runLoopControl();

//     inputs.wheelPosition = wheelMotor.getAngularPosition();
//     inputs.wheelVelocity = wheelMotor.getAngularVelocity();
//     inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getSetpoint());
//     inputs.wheelAppliedVolts = Volts.of(wheelMotor.getInputVoltage());
//     inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getCurrentDrawAmps());
//   }

//   private void runLoopControl() {
//     if (!wheelVoltageMode) {
//       wheelMotor.setInputVoltage(
//           wheelPID.calculate(wheelMotor.getAngularVelocity().in(RadiansPerSecond))
//               + wheelFF.calculate(wheelPID.getSetpoint()));
//     }
//   }

//   @Override
//   public void setWheelVelocitySetpoint(AngularVelocity velocity) {
//     this.wheelVoltageMode = false;
//     wheelPID.setSetpoint(velocity.in(RadiansPerSecond));
//   }

//   @Override
//   public void setWheelVoltage(Voltage volts) {
//     this.wheelVoltageMode = true;
//     wheelMotor.setInputVoltage(volts.in(Volts));
//   }

//   @Override
//   public void stopWheel() {
//     setWheelVoltage(Volts.of(0.0));
//   }
// }
