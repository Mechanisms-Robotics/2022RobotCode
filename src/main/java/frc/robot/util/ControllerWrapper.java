package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This acts as a wrapper class for the WPILib class XboxController. It's purpose is to provide a
 * nice interface because the XboxController buttons don't map to a PS4 Controller quite how you
 * would expect.
 */
public class ControllerWrapper {
  private static final double TRIGGER_THRESHOLD =
      0.1; // Threshold percentage to consider a trigger "pressed"

  public enum Direction {
    None,
    Up,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft
  }

  private final XboxController controller; // Instance of an XboxController object

  /**
   * Constructs a PS4Controller object.
   *
   * @param port The port that the controller is on in the DriverStation.
   */
  public ControllerWrapper(int port) {
    this.controller = new XboxController(port);
  }

  /**
   * Gets the X button's state from the controller.
   *
   * @return True or false depending on whether or not the X button is pressed.
   */
  public boolean getXButton() {
    return this.controller.getRawButton(1);
  }

  /**
   * Gets the Circle button's state from the controller.
   *
   * @return True or false depending on whether or not the Circle button is pressed.
   */
  public boolean getCircleButton() {
    return this.controller.getRawButton(2);
  }

  /**
   * Gets the Square button's state from the controller.
   *
   * @return True or false depending on whether or not the Square button is pressed.
   */
  public boolean getSquareButton() {
    return this.controller.getRawButton(3);
  }

  /**
   * Gets the Triangle button's state from the controller.
   *
   * @return True or false depending on whether or not the Triangle button is pressed.
   */
  public boolean getTriangleButton() {
    return this.controller.getRawButton(4);
  }

  /**
   * Gets the Left Bumper's state from the controller.
   *
   * @return True or false depending on whether or not the Left Bumper is pressed.
   */
  public boolean getLeftBumperButton() {
    return this.controller.getRawButton(5);
  }

  /**
   * Gets the Right Bumper's state from the controller.
   *
   * @return True or false depending on whether or not the Right Bumper is pressed.
   */
  public boolean getRightBumperButton() {
    return this.controller.getRawButton(6);
  }

  /**
   * Gets the Share button's state from the controller.
   *
   * @return True or false depending on whether or not the Share button is pressed.
   */
  public boolean getShareButton() {
    return this.controller.getRawButton(7);
  }

  /**
   * Gets the Options button's state from the controller.
   *
   * @return True or false depending on whether or not the Options button is pressed.
   */
  public boolean getOptionsButton() {
    return this.controller.getRawButton(8);
  }

  /**
   * Gets the Left Joystick button's state from the controller.
   *
   * @return True or false depending on whether or not the Left Joystick button is pressed.
   */
  public boolean getLeftJoystickButton() {
    return this.controller.getRawButton(9);
  }

  /**
   * Gets the Right Joystick button's state from the controller.
   *
   * @return True or false depending on whether or not the Right Joystick button is pressed.
   */
  public boolean getRightJoystickButton() {
    return this.controller.getRawButton(10);
  }

  /**
   * Gets the Left Trigger button's state from the controller.
   *
   * @return True or false depending on whether or not the Left Trigger button is pressed.
   */
  public boolean getLeftTriggerButton() {
    return (this.controller.getRawAxis(2) >= TRIGGER_THRESHOLD);
  }

  /**
   * Gets the Right Trigger button's state from the controller.
   *
   * @return True or false depending on whether or not the Right Trigger button is pressed.
   */
  public boolean getRightTriggerButton() {
    return (this.controller.getRawAxis(3) >= TRIGGER_THRESHOLD);
  }

  /**
   * Gets the Left Joystick's X axis value ranging from -1.0 to 1.0
   *
   * @return The Left Joystick's X axis value.
   */
  public double getLeftJoystickX() {
    return this.controller.getRawAxis(0);
  }

  /**
   * Gets the Left Joystick's Y axis value ranging from -1.0 to 1.0
   *
   * @return The Left Joystick's Y axis value.
   */
  public double getLeftJoystickY() {
    return -this.controller.getRawAxis(1);
  }

  /**
   * Gets the Left Trigger's state from the controller as a double ranging from 0.0 to 1.0
   *
   * @return The Left Trigger's state.
   */
  public double getLeftTrigger() {
    return this.controller.getRawAxis(2);
  }

  /**
   * Gets the Right Trigger's state from the controller as a double ranging from 0.0 to 1.0
   *
   * @return The Right Trigger's state.
   */
  public double getRightTrigger() {
    return this.controller.getRawAxis(3);
  }

  /**
   * Gets the Right Joystick's X axis value ranging from -1.0 to 1.0
   *
   * @return The Right Joystick's X axis value.
   */
  public double getRightJoystickX() {
    return this.controller.getRawAxis(4);
  }

  /**
   * Gets the Right Joystick's Y axis value ranging from -1.0 to 1.0
   *
   * @return The Right Joystick's Y axis value.
   */
  public double getRightJoystickY() {
    return -this.controller.getRawAxis(5);
  }

  /**
   * Gets the DPAD's state from the controller as a direction.
   *
   * @return The DPAD's state as a direction.
   */
  public Direction getPOV() {
    switch (this.controller.getPOV()) {
      case 0:
        return Direction.Up;
      case 45:
        return Direction.UpRight;
      case 90:
        return Direction.Right;
      case 135:
        return Direction.DownRight;
      case 180:
        return Direction.Down;
      case 225:
        return Direction.DownLeft;
      case 270:
        return Direction.Left;
      case 315:
        return Direction.UpLeft;
      default:
        return Direction.None;
    }
  }
}
