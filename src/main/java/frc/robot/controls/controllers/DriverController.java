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
    return -this.getFilteredAxis(Axis.LEFT_Y_AXIS);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(Axis.LEFT_X_AXIS);
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
    return this.getRawButton(RawButton.Y);
  }

  public boolean getWantsBrake() {
    return this.getRawButton(RawButton.LEFT_BUMPER);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(Axis.RIGHT_TRIGGER) > k_triggerActivationThreshold;
  }

  public boolean getWantsAutoAim() {
    return this.getRawButtonPressed(RawButton.START);
  }

  // SysId test mode //
  public boolean getWantsSysIdQuasistaticForward() {
    return this.getRawButtonPressed(RawButton.A);
  }

  public boolean getWantsSysIdQuasistaticBackward() {
    return this.getRawButtonPressed(RawButton.B);
  }

  public boolean getWantsSysIdDynamicForward() {
    return this.getRawButtonPressed(RawButton.X);
  }

  public boolean getWantsSysIdDynamicBackward() {
    return this.getRawButtonPressed(RawButton.Y);
  }

  public boolean getWantSysIdStop() {
    return this.getRawButtonPressed(RawButton.START);
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
    return this.getRawButtonPressed(RawButton.LEFT_BUMPER);
  }

  public boolean getWantsIntakeGround() {
    return this.getRawButtonPressed(RawButton.RIGHT_BUMPER);
  }

  public boolean getWantsIntake() {
    return this.getRawButton(RawButton.X);
  }

  public boolean getWantsEject() {
    return this.getRawButton(RawButton.B);
  }
}
