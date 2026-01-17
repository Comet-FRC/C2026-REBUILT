package frc.robot.util.controller;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CometLogitechController extends CometController {

  public CometLogitechController(int port) {
    super(new CommandGenericHID(port));
  }

  @Override
  public double getLeftY() {
    return super.controller.getRawAxis(1);
  }

  @Override
  public double getLeftX() {
    return super.controller.getRawAxis(0);
  }

  @Override
  public double getRightY() {
    return super.controller.getRawAxis(5);
  }

  @Override
  public double getRightX() {
    return super.controller.getRawAxis(4);
  }

  @Override
  public double getLeftTriggerAxis() {
    return super.controller.getRawAxis(2);
  }

  @Override
  public double getRightTriggerAxis() {
    return super.controller.getRawAxis(3);
  }

  @Override
  public Trigger a() {
    return super.controller.button(1);
  }

  @Override
  public Trigger b() {
    return super.controller.button(2);
  }

  @Override
  public Trigger x() {
    return super.controller.button(3);
  }

  @Override
  public Trigger y() {
    return super.controller.button(4);
  }

  @Override
  public Trigger leftBumper() {
    return super.controller.button(5);
  }

  @Override
  public Trigger rightBumper() {
    return super.controller.button(6);
  }

  @Override
  public Trigger leftMenu() {
    return super.controller.button(7);
  }

  @Override
  public Trigger rightMenu() {
    return super.controller.button(8);
  }

  @Override
  public Trigger leftStick() {
    return super.controller.button(9);
  }

  @Override
  public Trigger rightStick() {
    return super.controller.button(10);
  }

  @Override
  public Trigger leftTrigger() {
    return super.controller.axisGreaterThan(2, 0.5);
  }

  @Override
  public Trigger rightTrigger() {
    return super.controller.axisGreaterThan(3, 0.5);
  }

  @Override
  public Trigger up() {
    return super.controller.povUp();
  }

  @Override
  public Trigger left() {
    return super.controller.povLeft();
  }

  @Override
  public Trigger down() {
    return super.controller.povDown();
  }

  @Override
  public Trigger right() {
    return super.controller.povRight();
  }
}
