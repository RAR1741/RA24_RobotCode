package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Subsystem;

public class SwerveSysId extends Subsystem {
  private SwerveModule[] m_modules;

  public SwerveSysId(SwerveModule[] modules, String baseSmartDashboardKey) {
    super(baseSmartDashboardKey);
    m_modules = modules;
  }

  public Command sysIdTurnQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdTurnRoutine.quasistatic(direction);
  }

  public Command sysIdTurnDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdTurnRoutine.dynamic(direction);
  }

  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdDriveRoutine.quasistatic(direction);
  }

  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdDriveRoutine.dynamic(direction);
  }

  // Mutable holder for unit-safe SysID values (to avoid reallocation)
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdTurnRoutine = new SysIdRoutine( // i hate this constructor
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            for (SwerveModule module : m_modules) {
              module.sysidTurn(volts.in(Volts));
            }
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being characterized.
          log -> {
            // Record a frame for the front left
            log.motor("turn-frontleft")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.FRONT_LEFT].getTurnVelocity(),
                        MetersPerSecond));

            // Record a frame for the front right
            log.motor("turn-frontright")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.FRONT_RIGHT].getTurnVelocity(),
                        MetersPerSecond));

            // Record a frame for the back right
            log.motor("turn-backright")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.BACK_RIGHT].getTurnVelocity(),
                        MetersPerSecond));

            // Record a frame for the back left
            log.motor("turn-backleft")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_LEFT].getTurnMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.BACK_LEFT].getTurnPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.BACK_LEFT].getTurnVelocity(), MetersPerSecond));

          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in WPILog with this subsystem's name ("drive")
          this));

  private final SysIdRoutine m_sysIdDriveRoutine = new SysIdRoutine( // i hate this constructor
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            for (SwerveModule module : m_modules) {
              module.sysidDrive(volts.in(Volts));
            }
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being characterized.
          log -> {
            // Record a frame for the front left
            log.motor("drive-frontleft")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_LEFT].getDriveMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.FRONT_LEFT].getDrivePosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.FRONT_LEFT].getDriveVelocity(),
                        MetersPerSecond));

            // Record a frame for the front right
            log.motor("drive-frontright")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.FRONT_RIGHT].getDriveMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.FRONT_RIGHT].getDrivePosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.FRONT_RIGHT].getDriveVelocity(),
                        MetersPerSecond));

            // Record a frame for the back right
            log.motor("drive-backright")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_RIGHT].getDriveMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.BACK_RIGHT].getDrivePosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.BACK_RIGHT].getDriveVelocity(),
                        MetersPerSecond));

            // Record a frame for the back left
            log.motor("drive-backleft")
                .voltage(m_appliedVoltage.mut_replace(
                    m_modules[SwerveDrive.Module.BACK_LEFT].getDriveMotor().getAppliedOutput()
                        * RobotController.getBatteryVoltage(),
                    Volts))
                .linearPosition(
                    m_distance.mut_replace(m_modules[SwerveDrive.Module.BACK_LEFT].getDrivePosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_modules[SwerveDrive.Module.BACK_LEFT].getDriveVelocity(),
                        MetersPerSecond));

          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in WPILog with this subsystem's name ("drive")
          this));;

  public void outputTelemetry() {
  };

  public void reset() {
  };

  public void periodic() {
  };

  public void writePeriodicOutputs() {
  };

  public void stop() {
  };
}
