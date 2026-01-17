package frc.robot.util.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CometXboxController extends CometController {

  public CometXboxController(int port) {
    super(new CommandXboxController(port));
  }

  @Override
  public double getLeftY() {
    return ((CommandXboxController) controller).getLeftY();
  }

  @Override
  public double getLeftX() {
    return ((CommandXboxController) controller).getLeftX();
  }

  @Override
  public double getRightY() {
    return ((CommandXboxController) controller).getRightY();
  }

  @Override
  public double getRightX() {
    return ((CommandXboxController) controller).getRightX();
  }

  @Override
  public double getLeftTriggerAxis() {
    return ((CommandXboxController) controller).getLeftTriggerAxis();
  }

  @Override
  public double getRightTriggerAxis() {
    return ((CommandXboxController) controller).getRightTriggerAxis();
  }

  @Override
  public Trigger a() {
    return ((CommandXboxController) controller).a();
  }

  @Override
  public Trigger b() {
    return ((CommandXboxController) controller).b();
  }

  @Override
  public Trigger x() {
    return ((CommandXboxController) controller).x();
  }

  @Override
  public Trigger y() {
    return ((CommandXboxController) controller).y();
  }

  @Override
  public Trigger leftBumper() {
    return ((CommandXboxController) controller).leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return ((CommandXboxController) controller).rightBumper();
  }

  @Override
  public Trigger leftMenu() {
    return ((CommandXboxController) controller).back();
  }

  @Override
  public Trigger rightMenu() {
    return ((CommandXboxController) controller).start();
  }

  @Override
  public Trigger leftStick() {
    return ((CommandXboxController) controller).leftStick();
  }

  @Override
  public Trigger rightStick() {
    return ((CommandXboxController) controller).rightStick();
  }

  @Override
  public Trigger leftTrigger() {
    return ((CommandXboxController) controller).leftTrigger();
  }

  @Override
  public Trigger rightTrigger() {
    return ((CommandXboxController) controller).rightTrigger();
  }

  @Override
  public Trigger up() {
    return ((CommandXboxController) controller).povUp();
  }

  @Override
  public Trigger left() {
    return ((CommandXboxController) controller).povLeft();
  }

  @Override
  public Trigger right() {
    return ((CommandXboxController) controller).povRight();
  }

  @Override
  public Trigger down() {
    return ((CommandXboxController) controller).povDown();
  }
}
