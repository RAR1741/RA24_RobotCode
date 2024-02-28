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
    return this.getRawButton(Button.Y);
  }

  public boolean getWantsBrake() {
    return this.getRawButton(Button.LEFT_BUMPER);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER) > k_triggerActivationThreshold;
  }

  public boolean getWantsAutoAim() {
    return this.getRawButtonPressed(Button.START);
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

  public boolean getWantsIntakeStow() {
    return this.getRawButtonPressed(Button.LEFT_BUMPER);
  }

  public boolean getWantsIntakeGround() {
    return this.getRawButtonPressed(Button.RIGHT_BUMPER);
  }

  public boolean getWantsIntake() {
    return this.getRawButton(Button.X);
  }

  public boolean getWantsEject() {
    return this.getRawButton(Button.B);
  }
}
