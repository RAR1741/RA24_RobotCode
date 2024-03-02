package frc.robot.autonomous;

import frc.robot.autonomous.modes.AutoModeBase;
import frc.robot.autonomous.modes.CenterFourNote;
import frc.robot.autonomous.modes.CenterOneNoteMode;
import frc.robot.autonomous.modes.CenterTwoNoteMidringMode;
import frc.robot.autonomous.modes.DoNothingMode;
import frc.robot.autonomous.modes.LeftTwoNoteMode;
import frc.robot.autonomous.modes.RightTwoNoteMode;
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
    CENTER_ONE_NOTE,
    CENTER_TWO_NOTE,
    // TEST,
    // TEST2,
    CENTER_FOUR_NOTE,
    BOTRING_TWO_NOTE,
    PODIUM_TWO_NOTE
  }

  public Task getNextTask() {
    return m_autoMode.getNextTask();
  }

  public void setAutoMode(AutoMode mode) {
    switch (mode) {
      case DO_NOTHING:
        m_autoMode = new DoNothingMode();
        break;
      case CENTER_ONE_NOTE:
        m_autoMode = new CenterOneNoteMode();
        break;
      case CENTER_TWO_NOTE:
        m_autoMode = new CenterTwoNoteMidringMode();
        break;
      case CENTER_FOUR_NOTE:
        m_autoMode = new CenterFourNote();
        break;
      case BOTRING_TWO_NOTE:
        m_autoMode = new LeftTwoNoteMode();
        break;
      case PODIUM_TWO_NOTE:
        m_autoMode = new RightTwoNoteMode();
        break;
      // case TEST:
      // m_autoMode = new TestMode();
      // break;
      // case TEST2:
      // m_autoMode = new TestMode2();
      // break;
      default:
        System.out.println("Invalid auto mode selected. Defaulting to do nothing.");
        m_autoMode = new DoNothingMode();
        break;
    }

    m_autoMode.queueTasks();
  }
}
