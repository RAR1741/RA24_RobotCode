//Credit: https://gist.github.com/pordonj/970b2c189cc6ee06388b3e2f12abcb72

package frc.robot.controls.controllers;

public class DPadButton {

    FilteredController m_controller;
    Direction m_direction;

    public boolean m_pressed = false;

    public DPadButton(FilteredController controller, Direction direction) {
        this.m_controller = controller;
        this.m_direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        return m_controller.getPOV() == m_direction.direction;
    }

    public boolean getPressed() {
        if (get()) {
            if (!m_pressed) {
                m_pressed = true;
                return true;
            }
        } else {
            m_pressed = false;
        }
        return false;
    }

}
