package org.saxonrobotics.saxonutil.lib.io.motors.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.saxonrobotics.saxonutil.lib.io.motors.MotorIO;
import org.wpilib.units.measure.Angle;

public interface LinearSystemIO extends MotorIO {
  @AutoLog
  class LinearSystemIOInputs extends MotorIOInputs {
    public double positionRad;
    public double velocityRadPerSec;
  }

  default void updateInputs(LinearSystemIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setVelocity(double rps) {}

  default void resetPosition(Angle angle) {}
}
