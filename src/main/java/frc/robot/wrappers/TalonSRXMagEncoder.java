package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TalonSRXMagEncoder extends DutyCycleEncoder {
  public TalonSRXMagEncoder(int channel) {
    super(channel);

    // Based on logging:
    // - Ready: 230 ± 5
    setConnectedFrequencyThreshold(230);
  }
}
