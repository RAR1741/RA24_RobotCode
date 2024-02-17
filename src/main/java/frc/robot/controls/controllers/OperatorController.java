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
}
