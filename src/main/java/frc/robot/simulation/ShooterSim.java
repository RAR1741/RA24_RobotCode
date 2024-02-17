package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ShooterSim {
  private static ShooterSim m_sim = null;

  private final double k_simOffset = -90.0;

  private final DCMotor k_pivotMotor = DCMotor.getNeoVortex(1);
  private final double k_pivotGearRatio = 1.0;

  @SuppressWarnings("unused")
  private final SingleJointedArmSim m_joint = new SingleJointedArmSim(
      k_pivotMotor,
      k_pivotGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.Shooter.k_length),
          Constants.Shooter.k_mass),
      Constants.Shooter.k_length,
      Constants.Shooter.k_minAngle,
      Constants.Shooter.k_maxAngle,
      true,
      Constants.Shooter.k_startingAngle);

  private Mechanism2d m_mech2d = null;

  private final Translation2d m_origin = new Translation2d(
      (Constants.Simulation.k_width / 2) + Constants.Shooter.k_distanceFromCenter, 0);

  private MechanismRoot2d m_shooterBase = null;

  private MechanismLigament2d m_shooterPivot = null;

  private MechanismLigament2d m_shooter = null;

  public static ShooterSim getInstance(Mechanism2d mech2d) {
    if (m_sim == null) {
      m_sim = new ShooterSim(mech2d);
    }

    return m_sim;
  }

  private ShooterSim(Mechanism2d mech2d) {
    m_mech2d = mech2d;
    m_shooterBase = m_mech2d.getRoot("ShooterPivot", m_origin.getX(), Constants.Robot.k_bumperStart);
    m_shooterPivot = m_shooterBase.append(
        new MechanismLigament2d(
            "ShooterPivot",
            Constants.Shooter.k_pivotHeight,
            90,
            4,
            new Color8Bit(Color.kRed)));
    m_shooter = m_shooterPivot.append(
        new MechanismLigament2d(
            "Shooter",
            Constants.Shooter.k_length,
            k_simOffset,
            4,
            new Color8Bit(Color.kYellow)));
  }

  public void updateAngle(double angle) {
    m_shooter.setAngle(-(k_simOffset + angle));

    // Translation2d setpoint = m_origin.plus(new Translation2d(x, y));
    // m_crosshair.setPosition(setpoint.getX(), setpoint.getY());

    SmartDashboard.putNumber("Sim/Shooter Sim Angle", angle);
  }
}
