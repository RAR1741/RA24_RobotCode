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
    return -this.getFilteredAxis(1);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(0);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(4);
  }

  public double getSlowScaler() {
    return this.getFilteredAxis(3);
  }

  public double getBoostScaler() {
    return this.getFilteredAxis(2);
  }

  public boolean getWantsResetGyro() {
    return this.getRawButton(4);
  }

  public boolean getWantsBrake() {
    return this.getRawButton(5);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Strafe", getStrafeAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
