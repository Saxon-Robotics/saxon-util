package org.saxonrobotics.saxonutil.lib.io.motors.pivot;

import org.littletonrobotics.junction.AutoLog;
import org.saxonrobotics.saxonutil.lib.io.motors.MotorIO;
import org.wpilib.units.measure.Angle;

public interface PivotIO extends MotorIO {
  @AutoLog
  class PivotIOInputs extends MotorIOInputs {
    public double positionDeg;
    public double velocityDegPerSec;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setPosition(int slot, Angle angle) {
    setPosition(angle);
  }

  default void resetPosition(Angle angle) {}
}
