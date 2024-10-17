package frc.robot.controls.controllers;

public class DriverController extends FilteredController {
  public double k_triggerActivationThreshold = 0.5;

  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Drive
  public double getForwardAxis() {
    // return -this.getFilteredAxis(Axis.LEFT_Y_AXIS);
    return -this.getFilteredAxis(Axis.LEFT_Y_AXIS) * k_allianceMultiplier;
  }

  public double getStrafeAxis() {
    // return -this.getFilteredAxis(Axis.LEFT_X_AXIS);
    return -this.getFilteredAxis(Axis.LEFT_X_AXIS) * k_allianceMultiplier;
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(Axis.RIGHT_X_AXIS);
  }

  public double getSlowScaler() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }

  public double getBoostScaler() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  public boolean getWantsResetGyro() {
    return this.getRawButtonPressed(Button.START);
  }

  public boolean getWantsResetModules() {
    return this.getRawButtonPressed(Button.BACK);
  }

  // SysId test mode //
  public boolean getWantsSysIdQuasistaticForward() {
    return this.getRawButtonPressed(Button.A);
  }

  public boolean getWantsSysIdQuasistaticBackward() {
    return this.getRawButtonPressed(Button.B);
  }

  public boolean getWantsSysIdDynamicForward() {
    return this.getRawButtonPressed(Button.X);
  }

  public boolean getWantsSysIdDynamicBackward() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantSysIdStop() {
    return this.getRawButtonPressed(Button.START);
  }
  /////

  // Manual system test modes //
  public double testPositive() {
    return this.getFilteredAxis(Axis.LEFT_TRIGGER);
  }

  public double testNegative() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER);
  }
  /////

  public boolean getWantsIntakePivotToggle() {
    return this.getRawButtonPressed(Button.LEFT_BUMPER);
  }

  public boolean getWantsIntakeAutoFlipOverride() {
    return this.getRawButton(Button.LEFT_BUMPER);
  }

  public boolean getWantsIntake() {
    return this.getRawButton(Button.RIGHT_BUMPER);
  }

  public boolean getWantsStopIntake() {
    return this.getRawButtonReleased(Button.RIGHT_BUMPER);
  }

  public boolean getWantsEject() {
    return this.getRawButton(Button.B);
  }

  public boolean getWantsEjectFinished() {
    return this.getRawButtonReleased(Button.B);
  }

  public boolean getWantsShooterPass() {
    return this.getRawButton(Button.A);
  }

  public boolean getWantsAutoAim() {
    return this.getRawButton(Button.X);
  }

  public boolean getWantsEjectPivot() {
    return this.getRawButtonPressed(Button.Y);
  }

  public boolean getWantsTrap() {
    return this.getHat(Direction.RIGHT);
  }

  public boolean getWantsLLOn() {
    return this.getHatPressed(Direction.UP);
  }

  public boolean getWantsLLOff() {
    return this.getHatPressed(Direction.DOWN);
  }
}
