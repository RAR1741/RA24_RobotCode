package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverController extends FilteredController {
  private String m_smartDashboardKey = "DriverInput/";

  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  private final double k_triggerActivationThreshold = 0.5;

  // Drive
  public double getForwardAxis() {
    return -this.getFilteredAxis(RawAxis.LEFT_Y_AXIS);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(RawAxis.LEFT_X_AXIS);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(RawAxis.RIGHT_X_AXIS);
  }

  public double getSlowScaler() {
    return this.getFilteredAxis(RawAxis.RIGHT_TRIGGER);
  }

  public double getBoostScaler() {
    return this.getFilteredAxis(RawAxis.LEFT_TRIGGER);
  }

  public boolean getWantsResetGyro() {
    return this.getRawButton(RawButton.Y);
  }

  public boolean getWantsBrake() {
    return this.getRawButton(RawButton.LEFT_BUMPER);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(RawAxis.RIGHT_TRIGGER) > k_triggerActivationThreshold;
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Strafe", getStrafeAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
