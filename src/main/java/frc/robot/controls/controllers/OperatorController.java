package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsShoot() {
    return this.getRawButton(RawButton.X);
  }

  public double getWantsManualShooterPivot(double limit) {
    return (this.getFilteredAxis(Axis.RIGHT_TRIGGER) - this.getFilteredAxis(Axis.LEFT_TRIGGER)) * limit;
  }

  public boolean getWantsAmpAngle() {
    return this.getRawButtonPressed(RawButton.A);
  }

  public boolean getWantsSpeakerAngle() {
    return this.getRawButtonPressed(RawButton.Y);
  }

  public boolean getWantsShooterMaxAngle() {
    return this.getRawButtonPressed(RawButton.START);
  }

  public boolean getWantsShooterMinAngle() {
    return this.getRawButtonPressed(RawButton.BACK);
  }

  public boolean getWantsMaxSpeed() {
    return this.getHatPressed(Direction.UP);
  }

  public boolean getWantsHalfSpeed() {
    return this.getHatPressed(Direction.RIGHT);
  }

  public boolean getWantsQuarterSpeed() {
    return this.getHatPressed(Direction.LEFT);
  }

  public boolean getWantsStopped() {
    return this.getHatPressed(Direction.DOWN);
  }
}
