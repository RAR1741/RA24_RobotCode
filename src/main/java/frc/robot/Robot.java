package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.constants.ApolloConstants;
import frc.robot.constants.RobotConstants;
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
import frc.robot.subsystems.leds.LEDs;

public class Robot extends LoggedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(
      ApolloConstants.SwerveDrive.k_maxLinearAcceleration);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(
      ApolloConstants.SwerveDrive.k_maxLinearAcceleration);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(
      ApolloConstants.SwerveDrive.k_maxAngularAcceleration);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climber m_climber = Climber.getInstance();
  private final LEDs m_leds = LEDs.getInstance();

  // Auto tasks
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

  // Misc vars
  private final boolean k_lockHeading = true;
  private boolean m_intaking = false;
  public final static boolean k_ledsEnabled = true;

  private final Field m_field = Field.getInstance();

  @Override
  public void robotInit() {
    // Initialize on-board logging
    new RobotTelemetry();

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

    if (k_ledsEnabled) {
      m_allSubsystems.add(m_leds);
    }
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

    SmartDashboardController.logEncoderConnections();

    Logger.recordOutput("Robot Controller/CPU/Temperature", RobotController.getCPUTemp());

    // CommandScheduler.getInstance().run(); // used by sysid
  }

  @Override
  public void autonomousInit() {
    // m_swerve.resetGyro();
    m_swerve.setBrakeMode(false);
    m_swerve.resetTurnOffsets();

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_leds.setAllColor(Color.kBlue);
    } else {
      m_leds.setAllColor(Color.kRed);
    }

    m_swerve.m_limelightLeft.setLightEnabled(true);
    m_swerve.m_limelightRight.setLightEnabled(true);
    m_swerve.m_limelightShooter.setLightEnabled(true);

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
    m_swerve.resetRotationTarget();
    // m_swerve.resetAccelerometerPose();
    m_swerve.resetTurnOffsets();

    m_swerve.m_limelightLeft.setLightEnabled(false);
    m_swerve.m_limelightRight.setLightEnabled(false);
    m_swerve.m_limelightShooter.setLightEnabled(false);

    m_swerve.m_visionConstants = ApolloConstants.Vision.teleopVisionConstants;

    m_leds.breathe();
  }

  private void shooterCorrection() {
    if (m_shooter.getPreviousShooterAngle() > 60.0) {
      if (m_intake.getPivotTarget() == IntakePivotTarget.STOW && m_intake.isAtPivotTarget()) {
        m_shooter.setAngle(m_shooter.getPreviousShooterAngle());
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    double maxSpeed = ApolloConstants.SwerveDrive.k_maxSpeed + ((ApolloConstants.SwerveDrive.k_maxBoostSpeed -
        ApolloConstants.SwerveDrive.k_maxSpeed) * m_driverController.getBoostScaler());

    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis() * maxSpeed);
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis() * maxSpeed);
    double rot = m_rotRateLimiter
        .calculate(m_driverController.getTurnAxis() * ApolloConstants.SwerveDrive.k_maxAngularSpeed);

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = ApolloConstants.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - ApolloConstants.SwerveDrive.k_slowScaler));

    xSpeed *= slowScaler;
    ySpeed *= slowScaler;
    rot *= slowScaler;

    boolean wantsSpeakerAutoAim = m_driverController.getWantsAutoAim();
    boolean wantsAmpAutoAim = m_operatorController.getWantsAmpPivot();
    boolean wantsPassAutoAim = m_driverController.getWantsShooterPass();

    if (k_lockHeading) {
      m_swerve.driveLockedHeading(
          xSpeed, ySpeed, rot, true,
          wantsSpeakerAutoAim, wantsAmpAutoAim, wantsPassAutoAim);
    } else {
      // Automatically drive in a circle, to test swerve module things
      // double timeOffset = (Timer.getFPGATimestamp() % 10.0) / 10.0 * (2 * Math.PI);
      // double x = Math.cos(timeOffset);
      // double y = Math.sin(timeOffset);
      // m_swerve.drive(x, y, rot, false);

      m_swerve.drive(xSpeed, ySpeed, rot, true);
    }

    if (wantsSpeakerAutoAim) {
      m_shooter.setAngle(m_shooter.getSpeakerAutoAimAngle(m_swerve.getPose()));
      m_shooter.setSpeed(ShooterSpeedTarget.MAX);
      m_leds.setAllColor(Color.kBlue);
    } else if (wantsPassAutoAim) {
      m_shooter.setAngle(ApolloConstants.Shooter.k_passPivotAngle);
      m_shooter.setSpeed(ApolloConstants.Shooter.k_passRPM);
      m_leds.setAllColor(Color.kPurple);
    }

    if (m_driverController.getWantsResetGyro()) {
      m_swerve.resetGyro();
    }

    if (m_driverController.getWantsResetModules()) {
      m_swerve.resetTurnOffsets();
    }

    if (m_driverController.getWantsIntakePivotToggle()) {
      wantsAmpAutoAim = false;
      m_shooter.updatePreviousShooterAngle();

      if (m_intake.getPivotTarget() == IntakePivotTarget.STOW) {
        if (m_shooter.getPreviousShooterAngle() > 60.0) {
          m_shooter.setAngle(60.0);
        }

        m_intake.setPivotTarget(IntakePivotTarget.GROUND);
        m_intake.setIntakeState(IntakeState.INTAKE);

        m_intaking = true;
        m_leds.setAllColor(Color.kYellow);
      } else {
        m_intake.setPivotTarget(IntakePivotTarget.STOW);
        m_intake.setIntakeState(IntakeState.NONE);

        m_intaking = false;
      }
    }

    m_intake.overrideAutoFlip(m_driverController.getWantsIntakeAutoFlipOverride());

    shooterCorrection();

    if (m_driverController.getWantsEject()) {
      m_shooter.setAngle(ShooterPivotTarget.MIN);
      m_intake.setPivotTarget(IntakePivotTarget.EJECT);
    }

    if (wantsAmpAutoAim) {
      m_leds.redTwinkleFast();
      m_shooter.setAngle(ShooterPivotTarget.AMP);
      m_shooter.setSpeed(ShooterSpeedTarget.AMP);
    }

    if (m_driverController.getWantsTrap()) {
      m_shooter.setAngle(ShooterPivotTarget.TRAP);
      m_shooter.setSpeed(ShooterSpeedTarget.TRAP);
    }

    if (m_driverController.getWantsStopIntake()) {
      m_intaking = false;
      m_intake.setIntakeState(IntakeState.NONE);
    }

    if (m_driverController.getWantsIntake() && !m_intake.isHoldingNote()) {
      m_intake.setIntakeState(IntakeState.INTAKE);
      m_intaking = true;
    } else if(m_driverController.getWantsIntake() && m_intake.isHoldingNote()) {
      m_intake.setIntakeState(IntakeState.PULSE);
      m_intaking = false;
    } else if ((m_driverController.getWantsEject() || m_operatorController.getWantsEject()) &&
        (m_intake
            .getPivotAngle() < (RobotConstants.config.intake().k_stowPivotAngle
                - RobotConstants.config.intake().k_ejectPivotAngle))) {
      m_intake.setIntakeState(IntakeState.EJECT);
      m_intaking = false;
    } else if (m_operatorController.getWantsShoot() && m_intake.isAtStow()) {
      if (m_shooter.isAtTarget() && m_shooter.getPivotTarget() == ShooterPivotTarget.AMP) {
        RobotTelemetry.print("It's ampin' time!");
      }
      m_intake.setIntakeState(IntakeState.FEED_SHOOTER);
      m_intaking = false;
    } else if (!m_intaking) {
      m_intake.setIntakeState(IntakeState.NONE);
    }

    m_shooter.changePivotByAngle(m_operatorController.getWantsManualShooterPivot(0.1));

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
    } else if (m_operatorController.getWantsShooterBackwards()) {
      m_shooter.setSpeed(-1000.0); // we should never need this
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

    m_leds.rainbowBreatheSlow();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void disabledPeriodic() {
    m_leds.rainbowBreatheSlow();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_leds.rainbowChase();
    // m_swerve.m_limelightLeft.setLightEnabled(false);
    // m_swerve.m_limelightRight.setLightEnabled(false);
    // m_swerve.m_limelightShooter.setLightEnabled(false);
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
            .calculate(m_driverController.getTurnAxis() * ApolloConstants.SwerveDrive.k_maxAngularSpeed);
        double maxSpeed = ApolloConstants.SwerveDrive.k_maxSpeed + ((ApolloConstants.SwerveDrive.k_maxBoostSpeed -
            ApolloConstants.SwerveDrive.k_maxSpeed) * m_driverController.getBoostScaler());

        double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis() * maxSpeed);
        double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis() * maxSpeed);

        // slowScaler should scale between k_slowScaler and 1
        double slowScaler = ApolloConstants.SwerveDrive.k_slowScaler
            + ((1 - m_driverController.getSlowScaler()) * (1 - ApolloConstants.SwerveDrive.k_slowScaler));

        xSpeed *= slowScaler;
        ySpeed *= slowScaler;
        rot *= slowScaler;

        m_swerve.drive(xSpeed, ySpeed, rot, false);
        break;
      default:
        RobotTelemetry.print("you lost the game");
        break;
    }
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_swerve.getPose());
  }
}
