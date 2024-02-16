package frc.robot.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class SimMaster {
  private static SimMaster m_master = null;
  private IntakeSim m_intake = null;
  private ShooterSim m_shooter = null;

  private final Mechanism2d m_mech2d = new Mechanism2d(Constants.Simulation.k_width, Constants.Simulation.k_height);

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
    double bumperPosition = Constants.Simulation.k_width / 2 - Constants.Robot.k_length / 2;
    m_mech2d.getRoot("Robot", bumperPosition, Constants.Robot.k_bumperStart).append(
        new MechanismLigament2d(
            "RobotBase",
            Constants.Robot.k_length,
            0,
            Constants.Robot.k_bumperHeight,
            new Color8Bit(Color.kRed)));

    // Draw floor (because why not?)
    m_mech2d.getRoot("GroundPos", 0, 0).append(
        new MechanismLigament2d(
            "Floor",
            Constants.Simulation.k_width,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw amp
    double scoringPos = Constants.Simulation.k_width / 2 + Constants.Robot.k_length;
    m_mech2d.getRoot("AmpBottom", scoringPos, Constants.Field.k_ampBottom).append(
        new MechanismLigament2d(
            "AmpBottom",
            3.875,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpTop", scoringPos, Constants.Field.k_ampTop).append(
        new MechanismLigament2d(
            "AmpTop",
            3.875,
            0,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpFront", scoringPos, 0).append(
        new MechanismLigament2d(
            "AmpFrontWall",
            Constants.Field.k_ampBottom,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("AmpBack", scoringPos + 3.875, Constants.Field.k_ampBottom).append(
        new MechanismLigament2d(
            "AmpBackWall",
            Constants.Field.k_ampTop - Constants.Field.k_ampBottom,
            90,
            5,
            new Color8Bit(Color.kWhite)));

    // Draw speaker
    m_mech2d.getRoot("SpeakerBottom", scoringPos, Constants.Field.k_speakerBottom).append(
        new MechanismLigament2d(
            "SpeakerBottom",
            28,
            Constants.Field.k_speakerAngle,
            5,
            new Color8Bit(Color.kWhite)));

    m_mech2d.getRoot("SpeakerTop", scoringPos, Constants.Field.k_speakerTop).append(
        new MechanismLigament2d(
            "SpeakerTop",
            28,
            Constants.Field.k_speakerAngle,
            5,
            new Color8Bit(Color.kWhite)));

    double backX = 28.0 * Math.cos(Units.degreesToRadians(Constants.Field.k_speakerAngle));
    double backY = 28.0 * Math.sin(Units.degreesToRadians(Constants.Field.k_speakerAngle));
    m_mech2d.getRoot("SpeakerBack", scoringPos + backX, 0).append(
        new MechanismLigament2d(
            "SpeakerBack",
            Constants.Field.k_speakerTop + backY,
            90,
            5,
            new Color8Bit(Color.kWhite)));
  }
}
