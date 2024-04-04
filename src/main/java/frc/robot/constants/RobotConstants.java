package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;

public final class RobotConstants {

  public static String m_rioSerial = "empty";
  private static final double k_robotInitDelay = 2.0; // Seconds to wait before starting robot code

  public static ApolloConstants config = new ApolloConstants();

  public final String k_apolloSerial = "1234";
  public final String k_amadeusSerial = "4321";

  private RobotType m_robotType = null;
  private boolean m_intakeAttached = true;
  private boolean m_shooterAttached = true;
  private boolean m_climberAttached = true;

  public RobotConstants() {
    if (Robot.isReal()) {
      // Wait for the robot to fully boot up
      Timer.delay(RobotConstants.k_robotInitDelay);
    }

    if (RobotController.getSerialNumber() != null) {
      m_rioSerial = RobotController.getSerialNumber();
      RobotTelemetry.print("RIO SERIAL: " + m_rioSerial);
    }

    checkRobotType();
    switch (getRobotType()) {
      case SIM:
        // Set (riiiiiiiiiiiiiiiight the constants) all the constants (designed)
        // specifically for the simulation
        break;
      case APOLLO:
        break;
      case AMADEUS:
        m_intakeAttached = false;
        m_shooterAttached = false;
        m_climberAttached = false;
        break;
      default:
        // Set all the constants specifically for the robot
        break;
    }

    RobotTelemetry.print("ROBOT: " + getRobotType());
  }

  public RobotType checkRobotType() {
    if (Robot.isSimulation()) {
      m_robotType = RobotType.SIM;
      // config = new ApolloConstants();
      RobotTelemetry.print("Robot Type: Simulation");
    } else if (m_rioSerial.equals(k_apolloSerial)) {
      m_robotType = RobotType.APOLLO;
      // config = new ApolloConstants();
      RobotTelemetry.print("Robot Type: APOLLO 2024");
    } else if (m_rioSerial.equals(k_amadeusSerial)) {
      m_robotType = RobotType.AMADEUS;
      // config = new AmadeusConstants();
      RobotTelemetry.print("Robot Type: AMADEUS 2024");
    } else {
      m_robotType = RobotType.APOLLO;
      // config = new ApolloConstants();
      RobotTelemetry.print(RobotController.getSerialNumber());
      DriverStation.reportError(
          "Could not match rio to robot config; defaulting to APOLLO robot config",
          false);
      RobotTelemetry.print("Robot Type: APOLLO 2024");
    }
    return m_robotType;
  }

  public RobotType getRobotType() {
    return m_robotType;
  }

  public boolean isIntakeAttached() {
    return m_intakeAttached;
  }

  public boolean isShooterAttached() {
    return m_shooterAttached;
  }

  public boolean isClimberAttached() {
    return m_climberAttached;
  }

  public enum RobotType {
    SIM,
    APOLLO,
    AMADEUS
  }
}
