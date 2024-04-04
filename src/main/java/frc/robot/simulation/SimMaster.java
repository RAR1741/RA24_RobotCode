package frc.robot.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ApolloConstants;
import frc.robot.constants.RobotConstants;

public class SimMaster {
  private static SimMaster m_master = null;
  private IntakeSim m_intake = null;
  private ShooterSim m_shooter = null;

  private final Mechanism2d m_mech2d = new Mechanism2d(RobotConstants.config.simulation().k_width,
      RobotConstants.config.simulation().k_height);

  public static SimMaster getInstance() {
    if (m_master == null) {
      m_master = new SimMaster();
    }
    return m_master;
  }

  private SimMaster() {
    m_intake = IntakeSim.getInstance(m_mech2d);
    m_shooter = ShooterSim.getInstance(m_mech2d);

    addAdditionalDrawings();

    SmartDashboard.putData("Sim", m_mech2d);
  }

  public IntakeSim getIntakeSim() {
    return m_intake;
  }

  public ShooterSim getShooterSim() {
    return m_shooter;
  }

  private void addAdditionalDrawings() {
    // Draw the robot's bumpers
    double bumperPosition = RobotConstants.config.simulation().k_width / 2 - ApolloConstants.Robot.k_length / 2;
    m_mech2d.getRoot("Robot", bumperPosition, ApolloConstants.Robot.k_bumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            ApolloConstants.Robot.k_length,
            0,
            ApolloConstants.Robot.k_bumperHeight,
            new Color8Bit(Color.kRed)));

    // Draw floor (because why not?)
    m_mech2d.getRoot("GroundPos", 0, 0).append(
        new MechanismLigament2d(
            "Floor",
            RobotConstants.config.simulation().k_width,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw amp
    double scoringPos = RobotConstants.config.simulation().k_width / 2 + ApolloConstants.Robot.k_length;
    m_mech2d.getRoot("AmpBottom", scoringPos, RobotConstants.config.field().k_ampBottom).append(
        new MechanismLigament2d(
            "AmpBottom",
            3.875,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpTop", scoringPos, RobotConstants.config.field().k_ampTop).append(
        new MechanismLigament2d(
            "AmpTop",
            3.875,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpFront", scoringPos, 0).append(
        new MechanismLigament2d(
            "AmpFrontWall",
            RobotConstants.config.field().k_ampBottom,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpBack", scoringPos + 3.875, RobotConstants.config.field().k_ampBottom).append(
        new MechanismLigament2d(
            "AmpBackWall",
            RobotConstants.config.field().k_ampTop - RobotConstants.config.field().k_ampBottom,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw speaker
    m_mech2d.getRoot("SpeakerBottom", scoringPos, RobotConstants.config.field().k_speakerBottom).append(
        new MechanismLigament2d(
            "SpeakerBottom",
            28,
            RobotConstants.config.field().k_speakerAngle,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("SpeakerTop", scoringPos, RobotConstants.config.field().k_speakerTop).append(
        new MechanismLigament2d(
            "SpeakerTop",
            28,
            RobotConstants.config.field().k_speakerAngle,
            5,
            new Color8Bit(Color.kWhite)));

    double backX = 28.0 * Math.cos(Units.degreesToRadians(RobotConstants.config.field().k_speakerAngle));
    double backY = 28.0 * Math.sin(Units.degreesToRadians(RobotConstants.config.field().k_speakerAngle));
    m_mech2d.getRoot("SpeakerBack", scoringPos + backX, 0).append(
        new MechanismLigament2d(
            "SpeakerBack",
            RobotConstants.config.field().k_speakerTop + backY,
            90,
            5,
            new Color8Bit(Color.kWhite)));
  }
}
