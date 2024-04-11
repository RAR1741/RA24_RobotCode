package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDrive.Module;

public class SmartDashboardController {
  private static final SwerveDrive m_swerve = SwerveDrive.getInstance();

  public static void logEncoderConnections() {
    SmartDashboard.putBoolean("FL Connected", m_swerve.getModules()[Module.FRONT_LEFT].getAsbEncoderIsConnected());
    SmartDashboard.putBoolean("FR Connected", m_swerve.getModules()[Module.FRONT_RIGHT].getAsbEncoderIsConnected());
    SmartDashboard.putBoolean("BR Connected", m_swerve.getModules()[Module.BACK_RIGHT].getAsbEncoderIsConnected());
    SmartDashboard.putBoolean("BL Connected", m_swerve.getModules()[Module.BACK_LEFT].getAsbEncoderIsConnected());
  }
}
