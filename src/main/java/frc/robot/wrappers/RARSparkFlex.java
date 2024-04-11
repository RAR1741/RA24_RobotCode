package frc.robot.wrappers;

import com.revrobotics.CANSparkFlex;

public class RARSparkFlex extends CANSparkFlex {
  public RARSparkFlex(int deviceID, MotorType type) {
    super(deviceID, type);

    /**
     * On the REVLib doc pages, it explains what each of the CAN status frames
     * communicate.
     * https://docs.revrobotics.com/brushless/spark-max/control-interfaces
     *
     * We are currently never using these encoders through the SparkMax Data port:
     * - Analog Encoder
     * - Alternative Encoder
     * - Duty Cycle Encoders
     *
     * These status frames are fine to occur less often, every second (1000ms) or
     * so.
     */
    setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000); // Analog Sensor
    setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000); // Alternative Encoder
    setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000); // Duty Cycle Encoder
    setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000); // Duty Cycle Encoder
  }
}
