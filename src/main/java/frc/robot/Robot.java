package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.AutoRunner.AutoMode;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.climber.Climbers;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends TimedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(3);

  // PID Controller for Swerve Auto Aim speed
  private final PIDController m_autoAimPID = new PIDController(
      Constants.SwerveDrive.AutoAim.k_P,
      Constants.SwerveDrive.AutoAim.k_I,
      Constants.SwerveDrive.AutoAim.k_D);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climbers m_climbers = Climbers.getInstance();

  // Auto tasks
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  private Timer m_timer = new Timer();

  private boolean m_autoAimEnabled = false;

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

  private final Field m_field = Field.getInstance();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    DataLogManager.start();
    System.out.println("Logging initialized. Fard.");

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
    // m_allSubsystems.add(m_intake);
    // m_allSubsystems.add(m_shooter);
    // m_allSubsystems.add(m_climbers);

    m_swerve.setGyroAngleAdjustment(0);
  }

  @Override
  public void robotPeriodic() {
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
    m_autoRunner.setAutoMode(AutoMode.TEST);
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

    // if (m_driverController.getWantsAutoAim() && m_swerve.getPose().getX() >=
    // Constants.Field.k_autoAimThreshold && !autoAimEnabled) {
    // autoAimEnabled = true;
    // }
    // if (autoAimEnabled && m_driverController.getWantsAutoAim() ||
    // m_swerve.getPose().getX() <= Constants.Field.k_autoAimThreshold) {
    // autoAimEnabled = false;
    // }

    if (m_autoAimEnabled) {
      rot = m_autoAimPID.calculate(m_swerve.getRotation2d().getRadians(), m_swerve.calculateAutoAimAngle(false));
    } else {
      rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis());
    }

    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis());
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis());

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = Constants.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.SwerveDrive.k_slowScaler));

    // boostScaler should scale between 1 and k_boostScaler
    double boostScaler = 1 + (m_driverController.getBoostScaler() * (Constants.SwerveDrive.k_boostScaler - 1));

    xSpeed *= slowScaler * boostScaler;
    ySpeed *= slowScaler * boostScaler;
    rot *= slowScaler * boostScaler;

    m_swerve.drive(xSpeed, ySpeed, rot, true);

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    if (m_driverController.getWantsAutoAim()) {
      m_autoAimEnabled = !m_autoAimEnabled;
    }

    m_driverController.outputTelemetry();

    if (m_timer.get() == 0) {
      m_timer.start();
    }

    double a = Helpers.modDegrees(m_timer.get() * 5);
    m_intake.setSimPosition(a);
    m_shooter.setSimPosition(a);
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void disabledPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // m_swerve.drive(0, 0, 0, false);

    switch(Preferences.getString("Test Mode","NONE")) {
      case "SYSID_SWERVE":
        if (m_driverController.getWantsSysIdQuasistaticForward()) {
          m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdQuasistaticBackward()) {
          m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
        } else if (m_driverController.getWantsSysIdDynamicForward()) {
          m_swerve.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
        } else if (m_driverController.getWantsSysIdDynamicBackward()) {
          m_swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
        }
        break;
      case "INTAKE_TEST_MODE":
        m_intake.manualPivotControl(m_driverController.intakeTestAxisPositive(), m_driverController.intakeTestAxisNegative(), 0.25);
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
}
