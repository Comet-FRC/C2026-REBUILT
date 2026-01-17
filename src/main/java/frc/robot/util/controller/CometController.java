package frc.robot.util.controller;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * @see https://gist.github.com/palmerj/586375bcc5bc83ccdaf00c6f5f863e86
 */
public abstract class CometController {

  final CommandGenericHID controller;

  CometController(CommandGenericHID controller) {
    this.controller = controller;
  }

  public CommandGenericHID getHid() {
    return this.controller;
  }

  public abstract double getLeftY();

  public abstract double getLeftX();

  public abstract double getRightY();

  public abstract double getRightX();

  public abstract double getLeftTriggerAxis();

  public abstract double getRightTriggerAxis();

  public abstract Trigger a();

  public abstract Trigger b();

  public abstract Trigger x();

  public abstract Trigger y();

  public abstract Trigger leftBumper();

  public abstract Trigger rightBumper();

  public abstract Trigger leftMenu();

  public abstract Trigger rightMenu();

  public abstract Trigger leftStick();

  public abstract Trigger rightStick();

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public abstract Trigger leftTrigger();

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public abstract Trigger rightTrigger();

  public abstract Trigger up();

  public abstract Trigger left();

  public abstract Trigger down();

  public abstract Trigger right();
}
