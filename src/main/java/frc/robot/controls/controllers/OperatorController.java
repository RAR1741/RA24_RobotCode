package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.Preferences;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsShoot() {
    return this.getRawButton(Button.A);
  }

  public boolean getWantsEject() {
    return this.getRawButton(Button.B);
  }

  public boolean getWantsEjectFinished() {
    return this.getRawButtonReleased(Button.B);
  }

  public double getWantsManualShooterPivot(double limit) {
    return (this.getFilteredAxis(Axis.RIGHT_TRIGGER) - this.getFilteredAxis(Axis.LEFT_TRIGGER)) * limit;
  }

  public boolean getWantsAmpPivot() {
    return this.getRawButtonPressed(Button.X);
  }

  public boolean getWantsSubwooferAngle() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantsShooterMaxAngle() {
    return this.getRawButtonPressed(Button.START);
  }

  public boolean getWantsShooterMinAngle() {
    return this.getRawButtonPressed(Button.BACK);
  }

  public boolean getWantsMaxSpeed() {
    return this.getRawButtonPressed(Button.RIGHT_BUMPER);
  }

  public boolean getWantsNoSpeed() {
    return this.getRawButtonPressed(Button.LEFT_BUMPER);
  }

  public boolean getWantsClimberLower() {
    return this.getHat(Direction.DOWN);
  }

  public boolean getWantsClimberTiltRight() {
    return this.getHat(Direction.RIGHT);
  }

  public boolean getWantsClimberRaise() {
    return this.getHat(Direction.UP);
  }

  public boolean getWantsClimberTiltLeft() {
    return this.getHat(Direction.LEFT);
  }

  public boolean getWantsShooterOffDemo() {
    return this.getRawButtonPressed(Button.LEFT_JOYSTICK);
  }

  public boolean getWantsDemoLEDCycle() {
    if(!Preferences.getBoolean("Demo Mode", false)){
      return false;
    }
    return this.getRawButtonPressed(Button.RIGHT_JOYSTICK);
  }
}
