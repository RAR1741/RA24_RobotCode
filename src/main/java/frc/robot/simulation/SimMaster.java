package frc.robot.simulation;

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
  }
}
