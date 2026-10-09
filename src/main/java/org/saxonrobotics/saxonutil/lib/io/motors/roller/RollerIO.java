package org.saxonrobotics.saxonutil.lib.io.motors.roller;

import org.littletonrobotics.junction.AutoLog;
import org.saxonrobotics.saxonutil.lib.io.motors.MotorIO;

public interface RollerIO extends MotorIO {
  @AutoLog
  class RollerIOInputs extends MotorIOInputs {
    public double velocityRPS;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void setVelocity(double rps) {}
}
