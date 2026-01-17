package frc.robot.util.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CometPS4Controller extends CometController {

  public CometPS4Controller(int port) {
    super(new CommandPS4Controller(port));
  }

  @Override
  public double getLeftY() {
    return ((CommandPS4Controller) controller).getLeftY();
  }

  @Override
  public double getLeftX() {
    return ((CommandPS4Controller) controller).getLeftX();
  }

  @Override
  public double getRightY() {
    return ((CommandPS4Controller) controller).getRightY();
  }

  @Override
  public double getRightX() {
    return ((CommandPS4Controller) controller).getRightX();
  }

  @Override
  public double getLeftTriggerAxis() {
    return ((CommandPS4Controller) controller).getL2Axis();
  }

  @Override
  public double getRightTriggerAxis() {
    return ((CommandPS4Controller) controller).getR2Axis();
  }

  @Override
  public Trigger a() {
    return ((CommandPS4Controller) controller).cross();
  }

  @Override
  public Trigger b() {
    return ((CommandPS4Controller) controller).circle();
  }

  @Override
  public Trigger x() {
    return ((CommandPS4Controller) controller).square();
  }

  @Override
  public Trigger y() {
    return ((CommandPS4Controller) controller).triangle();
  }

  @Override
  public Trigger leftBumper() {
    return ((CommandPS4Controller) controller).L1();
  }

  @Override
  public Trigger rightBumper() {
    return ((CommandPS4Controller) controller).R1();
  }

  @Override
  public Trigger leftMenu() {
    return ((CommandPS4Controller) controller).share();
  }

  @Override
  public Trigger rightMenu() {
    return ((CommandPS4Controller) controller).options();
  }

  @Override
  public Trigger leftStick() {
    return ((CommandPS4Controller) controller).L3();
  }

  @Override
  public Trigger rightStick() {
    return ((CommandPS4Controller) controller).R3();
  }

  @Override
  public Trigger leftTrigger() {
    return ((CommandPS4Controller) controller).L2();
  }

  @Override
  public Trigger rightTrigger() {
    return ((CommandPS4Controller) controller).R2();
  }

  @Override
  public Trigger up() {
    return ((CommandPS4Controller) controller).povUp();
  }

  @Override
  public Trigger left() {
    return ((CommandPS4Controller) controller).povLeft();
  }

  @Override
  public Trigger right() {
    return ((CommandPS4Controller) controller).povRight();
  }

  @Override
  public Trigger down() {
    return ((CommandPS4Controller) controller).povDown();
  }
}
