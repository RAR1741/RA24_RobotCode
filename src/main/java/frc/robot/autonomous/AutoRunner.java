package frc.robot.autonomous;

import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.DefaultMode;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.FourNoteAutoTargetMode;
import frc.robot.autonomous.modes.FourNoteMode;
import frc.robot.autonomous.modes.ShootMidringRing4;
import frc.robot.autonomous.modes.TestMode;
import frc.robot.autonomous.modes.TestMode2;
import frc.robot.autonomous.tasks.Task;

public class AutoRunner {
  private static AutoRunner m_autoRunner = null;
  private AutoModeBase m_autoMode;

  public static AutoRunner getInstance() {
    if (m_autoRunner == null) {
      m_autoRunner = new AutoRunner();
    }
    return m_autoRunner;
  }

  public enum AutoMode {
    DO_NOTHING,
    DEFAULT,
    SHOOT_MIDRING_RING_4,
    TEST,
    TEST2,
    FOUR_NOTE,
    FOUR_NOTE_AUTO_TARGET
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    switch (mode) {
      case DO_NOTHING:
        m_autoMode = new DoNothingMode();
        break;
      case DEFAULT:
        m_autoMode = new DefaultMode();
        break;
      case SHOOT_MIDRING_RING_4:
        m_autoMode = new ShootMidringRing4();
        break;
      case TEST:
        m_autoMode = new TestMode();
        break;
      case TEST2:
        m_autoMode = new TestMode2();
        break;
      case FOUR_NOTE:
        m_autoMode = new FourNoteMode();
        break;
      case FOUR_NOTE_AUTO_TARGET:
        m_autoMode = new FourNoteAutoTargetMode();
        break;
      default:
        System.out.println("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
        break;
    }

    m_autoMode.queueTasks();
  }
}
