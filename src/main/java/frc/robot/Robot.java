package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  // PID Controller for Swerve Auto Aim speed
  private final PIDController m_aimPID = new PIDController(
      Constants.SwerveDrive.AutoAim.k_P,
      Constants.SwerveDrive.AutoAim.k_I,
      Constants.SwerveDrive.AutoAim.k_D);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  // private final Climbers m_climbers = Climbers.getInstance();

  // Auto tasks
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  private boolean m_autoAimEnabled = false;

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

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
    // m_allSubsystems.add(m_climbers);

    m_swerve.setGyroAngleAdjustment(0);
  }

  @Override
  public void robotPeriodic() {
    // if (!(Preferences.getString("Test Mode", "NONE").contains("SYSID") &&
    // DriverStation.isTest())) {
    // }
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();

    CommandScheduler.getInstance().run(); // used by sysid
  }

  @Override
  public void autonomousInit() {
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

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    m_swerve.setBrakeMode(false);
    m_swerve.drive(0, 0, 0, false);
  }

  @Override
  public void teleopPeriodic() {
    double rot = 0.0;
    boolean lockHeading = false;

    // if (m_driverController.getWantsAutoAim() && m_swerve.getPose().getX() >=
    // Constants.Field.k_autoAimThreshold && !autoAimEnabled) {
    // autoAimEnabled = true;
    // }
    // if (autoAimEnabled && m_driverController.getWantsAutoAim() ||
    // m_swerve.getPose().getX() <= Constants.Field.k_autoAimThreshold) {
    // autoAimEnabled = false;
    // }

    if (m_driverController.getTurnAxis() == 0.0) {
      lockHeading = true;
    }

    if (m_autoAimEnabled) {
      rot = m_aimPID.calculate(m_swerve.getRotation2d().getRadians(), m_swerve.calculateAutoAimAngle(false));
    } else {
      rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis() * Constants.SwerveDrive.k_maxAngularSpeed);
    }

    double maxSpeed = m_driverController.getBoostScaler() > 0.5 ? Constants.SwerveDrive.k_maxBoostSpeed
        : Constants.SwerveDrive.k_maxSpeed; // TODO: Use axis to be able to dial in boost speed

    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis() * maxSpeed);
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis() * maxSpeed);

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = Constants.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.SwerveDrive.k_slowScaler));

    xSpeed *= slowScaler;
    ySpeed *= slowScaler;
    rot *= slowScaler;

    m_swerve.drive(xSpeed, ySpeed, rot, true);

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    if (m_driverController.getWantsAutoAim()) {
      m_autoAimEnabled = !m_autoAimEnabled;
    }

    if (m_driverController.getWantsIntakeStow()) {
      m_intake.setPivotTarget(IntakePivotTarget.STOW);
    }

    if (m_driverController.getWantsIntakeGround()) {
      m_intake.setPivotTarget(IntakePivotTarget.GROUND);
    }

    if (m_driverController.getWantsIntake()) {
      m_intake.setState(IntakeState.INTAKE);
    } else if (m_driverController.getWantsEject()) {
      m_intake.setState(IntakeState.EJECT);
    } else if (m_operatorController.getWantsShoot() && m_intake.isAtPivotTarget(IntakePivotTarget.STOW)) {
      m_intake.setState(IntakeState.FEED_SHOOTER);
    } else {
      m_intake.setState(IntakeState.NONE);
    }

    m_shooter.changePivotByAngle(m_operatorController.getWantsManualShooterPivot(0.5));

    if (m_operatorController.getWantsAmpAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.AMP);
    }

    if (m_operatorController.getWantsSpeakerAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.SPEAKER);
    }

    if (m_operatorController.getWantsShooterMaxAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.MAX);
    }

    if (m_operatorController.getWantsShooterMinAngle()) {
      m_shooter.setAngle(ShooterPivotTarget.MIN);
    }

    if (m_operatorController.getWantsMaxSpeed()) {
      m_shooter.setSpeed(ShooterSpeedTarget.MAX);
    } else if (m_operatorController.getWantsHalfSpeed()) {
      m_shooter.setSpeed(ShooterSpeedTarget.HALF);
    } else if (m_operatorController.getWantsQuarterSpeed()) {
      m_shooter.setSpeed(ShooterSpeedTarget.QUARTER);
    } else if (m_operatorController.getWantsStopped()) {
      m_shooter.setSpeed(ShooterSpeedTarget.OFF);
    }

  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
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
          m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdQuasistaticBackward()) {
          m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (m_driverController.getWantsSysIdDynamicForward()) {
          m_swerve.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdDynamicBackward()) {
          m_swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
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
        // m_climber.manualControl();
        break;
      default:
        System.out.println("you lost the game");
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
    Logger.recordMetadata("ProjectName", "TBD Robot Name");
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
