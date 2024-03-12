package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends LoggedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(Constants.SwerveDrive.k_maxLinearAcceleration);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(Constants.SwerveDrive.k_maxLinearAcceleration);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(Constants.SwerveDrive.k_maxAngularAcceleration);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climber m_climber = Climber.getInstance();

  // Auto tasks
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

  // Misc vars
  private boolean m_lockHeading = true;
  private boolean m_intaking = false;

  private final Field m_field = Field.getInstance();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    setupLogging();

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    // Camera server
    // m_camera = CameraServer.startAutomaticCapture();

    if (!RobotBase.isReal()) {
      Preferences.setDouble("SwerveDrive/x", 0);
      Preferences.setDouble("SwerveDrive/y", 0);
      Preferences.setDouble("SwerveDrive/rot", 0);
    }

    Preferences.initString("Test Mode", "NONE");

    m_allSubsystems.add(m_swerve);
    m_allSubsystems.add(m_intake);
    m_allSubsystems.add(m_shooter);
    m_allSubsystems.add(m_climber);
  }

  @Override
  public void robotPeriodic() {
    if (!(Preferences.getString("Test Mode", "NONE").contains("SYSID") && DriverStation.isTest())) {
      m_allSubsystems.forEach(subsystem -> subsystem.periodic());
      m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    }
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();
    m_swerve.setAllianceGyroAngleAdjustment();
    m_driverController.setAllianceMultiplier();
    m_operatorController.setAllianceMultiplier();

    // CommandScheduler.getInstance().run(); // used by sysid
  }

  @Override
  public void autonomousInit() {
    // m_swerve.resetGyro();
    m_swerve.setBrakeMode(false);

    m_autoRunner.setAutoMode(m_autoChooser.getSelectedAuto());
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      m_shooter.setAngle(m_shooter.getSpeakerAutoAimAngle(m_swerve.getPose()));

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          // m_swerve.m_limelightLeft.setLightEnabled(!m_swerve.m_limelightLeft.getLightEnabled());
          // m_swerve.m_limelightRight.setLightEnabled(!m_swerve.m_limelightRight.getLightEnabled());
          // m_swerve.m_limelightShooter.setLightEnabled(!m_swerve.m_limelightShooter.getLightEnabled());

          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    // m_swerve.resetGyro();
    m_swerve.setBrakeMode(false);
    m_swerve.drive(0, 0, 0, false);
  }

  boolean m_wantsAmpAutoAim = false;

  @Override
  public void teleopPeriodic() {
    double maxSpeed = Constants.SwerveDrive.k_maxSpeed + ((Constants.SwerveDrive.k_maxBoostSpeed -
        Constants.SwerveDrive.k_maxSpeed) * m_driverController.getBoostScaler());

    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis() * maxSpeed);
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis() * maxSpeed);
    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis() * Constants.SwerveDrive.k_maxAngularSpeed);

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = Constants.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.SwerveDrive.k_slowScaler));

    xSpeed *= slowScaler;
    ySpeed *= slowScaler;
    rot *= slowScaler;

    boolean wantsSpeakerAutoAim = m_driverController.getWantsAutoAim();
    m_wantsAmpAutoAim = m_driverController.getWantsAmpPivot();

    if (m_lockHeading) {
      m_swerve.driveLockedHeading(xSpeed, ySpeed, rot, true, wantsSpeakerAutoAim, m_wantsAmpAutoAim);
    } else {
      m_swerve.drive(xSpeed, ySpeed, rot, true);
    }

    if (wantsSpeakerAutoAim) {
      m_shooter.setAngle(m_shooter.getSpeakerAutoAimAngle(m_swerve.getPose()));
    }

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    if (m_driverController.getWantsResetModules()) {
      m_swerve.resetTurnOffsets();
    }

    if (m_driverController.getWantsIntakePivotToggle()) {
      m_wantsAmpAutoAim = false;
      if (m_intake.getPivotTarget() == IntakePivotTarget.STOW) {
        m_intake.setPivotTarget(IntakePivotTarget.GROUND);
        m_intake.setIntakeState(IntakeState.INTAKE);
        m_intaking = true;
      } else {
        m_intake.setPivotTarget(IntakePivotTarget.STOW);
        m_intake.setIntakeState(IntakeState.NONE);
        m_intaking = false;
      }
    }

    if (m_wantsAmpAutoAim) {
      m_shooter.setAngle(ShooterPivotTarget.MIN);
      m_intake.setPivotTarget(IntakePivotTarget.AMP);
    }

    if (m_driverController.getWantsStopIntake()) {
      m_intaking = false;
      m_intake.setIntakeState(IntakeState.NONE);
    }

    if (m_driverController.getWantsIntake() && !m_intake.isHoldingNote()) {
      m_intake.setIntakeState(IntakeState.INTAKE);
      m_intaking = true;
    } else if (m_driverController.getWantsEject() || m_operatorController.getWantsEject()) {
      m_intake.setIntakeState(IntakeState.EJECT);
      if (m_intake.isAtPivotTarget() && m_intake.getPivotTarget() == IntakePivotTarget.AMP) {
        System.out.println("It's ampin' time"); // -andy
      }
      m_intaking = false;
    } else if (m_operatorController.getWantsShoot() &&
        m_intake.isAtPivotTarget() &&
        m_intake.getPivotTarget() == IntakePivotTarget.STOW) {
      m_intake.setIntakeState(IntakeState.FEED_SHOOTER);
      m_intaking = false;
    } else if (!m_intaking) {
      m_intake.setIntakeState(IntakeState.NONE);
    }

    if (m_driverController.getWantsEjectPivot()) {
      m_intake.setPivotTarget(IntakePivotTarget.EJECT);
      m_intaking = false;
    }

    m_shooter.changePivotByAngle(m_operatorController.getWantsManualShooterPivot(0.5));

    if (m_operatorController.getWantsPodiumAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.PODIUM);
    }

    if (m_operatorController.getWantsSubwooferAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.SUBWOOFER);
    }

    if (m_operatorController.getWantsShooterMaxAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.MAX);
    }

    if (m_operatorController.getWantsShooterMinAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    }

    if (m_operatorController.getWantsMaxSpeed()) {
      m_shooter.setSpeed(ShooterSpeedTarget.MAX);
    } else if (m_operatorController.getWantsNoSpeed()) {
      m_shooter.setSpeed(ShooterSpeedTarget.OFF);
    }

    if (m_operatorController.getWantsClimberRaise()) {
      m_climber.raise();
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    } else if (m_operatorController.getWantsClimberLower()) {
      m_climber.lower();
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    } else if (m_operatorController.getWantsClimberTiltLeft()) {
      m_climber.tiltLeft();
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    } else if (m_operatorController.getWantsClimberTiltRight()) {
      m_climber.tiltRight();
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    } else {
      m_climber.stopClimber();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());

    m_swerve.m_limelightLeft.setLightEnabled(false);
    m_swerve.m_limelightRight.setLightEnabled(false);
    m_swerve.m_limelightShooter.setLightEnabled(false);
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    switch (Preferences.getString("Test Mode", "NONE")) {
      case "SYSID":
        m_intake.stopIntake();

        if (m_driverController.getWantsSysIdQuasistaticForward()) {
          m_swerve.sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdQuasistaticBackward()) {
          m_swerve.sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (m_driverController.getWantsSysIdDynamicForward()) {
          m_swerve.sysIdDriveDynamic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdDynamicBackward()) {
          m_swerve.sysIdDriveDynamic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (m_driverController.getWantSysIdStop()) {
          CommandScheduler.getInstance().cancelAll();
        }
        break;
      case "INTAKE_PIVOT":
        m_intake.manualPivotControl(m_driverController.testPositive(), m_driverController.testNegative(), 0.25);
        break;
      case "INTAKE_INTAKE":
        m_intake.manualIntakeControl(m_driverController.testPositive(), m_driverController.testNegative(), 0.25);
        break;
      case "SHOOTER_PIVOT":
        m_shooter.manualPivotControl(m_driverController.testPositive(), m_driverController.testNegative(), 0.4);
        break;
      case "SHOOTER_SHOOT":
        m_shooter.manualShootControl(m_driverController.testPositive(), m_driverController.testNegative(), 0.85);
        break;
      case "CLIMBER":
        m_shooter.setAngle(ShooterPivotTarget.MIN);
        // m_climber.manualControl(m_driverController.testPositive(),
        // m_driverController.testNegative(), 0.10);
        break;
      case "POINT_FORWARD":
        m_swerve.pointModules(0, 0, 0, false);
        break;
      case "NO_GYRO_DRIVE":
        double rot = m_rotRateLimiter
            .calculate(m_driverController.getTurnAxis() * Constants.SwerveDrive.k_maxAngularSpeed);
        double maxSpeed = Constants.SwerveDrive.k_maxSpeed + ((Constants.SwerveDrive.k_maxBoostSpeed -
            Constants.SwerveDrive.k_maxSpeed) * m_driverController.getBoostScaler());

        double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis() * maxSpeed);
        double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis() * maxSpeed);

        // slowScaler should scale between k_slowScaler and 1
        double slowScaler = Constants.SwerveDrive.k_slowScaler
            + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.SwerveDrive.k_slowScaler));

        xSpeed *= slowScaler;
        ySpeed *= slowScaler;
        rot *= slowScaler;

        m_swerve.drive(xSpeed, ySpeed, rot, false);
        break;
      default:
        // System.out.println("you lost the game"); jacob why
        break;
    }
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_swerve.getPose());
  }

  public void setLEDs() {
  }

  @SuppressWarnings("resource")
  private void setupLogging() {
    Logger.recordMetadata("ProjectName", "Apollo, Eater of Batteries");
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    // if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // }

    // TODO: figure out log replaying, as this is super powerful
    // else {
    // setUseTiming(false); // Run as fast as possible
    // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
    // AdvantageScope (or prompt the user)
    // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    // "_sim"))); // Save outputs to a new log
    // }

    Logger.start();

    System.out.println("Logging initialized. Fard.");
  }
}
