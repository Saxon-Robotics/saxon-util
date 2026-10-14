package org.saxonrobotics.saxonutil.lib.io.motors.pivot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.saxonrobotics.saxonutil.lib.io.motors.Motor;
import org.saxonrobotics.saxonutil.lib.io.motors.MotorIO;
import org.saxonrobotics.saxonutil.lib.io.motors.elevator.LinearSystem;
import org.saxonrobotics.saxonutil.lib.io.sensors.encoder.EncoderIO;
import org.saxonrobotics.saxonutil.lib.subsystems.SubsystemManager;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class Pivot extends Motor<PivotIO, PivotIOInputsAutoLogged> {
  private Transform3d pivotOffset;

  public Pivot(String name, PivotIO io, EncoderIO encoderIO, BooleanSupplier brakeMode) {
    super(name, io, new PivotIOInputsAutoLogged(), encoderIO, brakeMode);
    io.configure(true, false);
  }

  public Pivot(String name, PivotIO io, EncoderIO encoderIO) {
    this(name, io, encoderIO, SubsystemManager.getInstance()::isRobotEnabled);
  }

  public Pivot(String name, PivotIO io) {
    this(name, io, in -> {});
  }

  /**
   * Set up visualization for this mechanism.
   *
   * @param offset Pivot offset. If this is set to null, visualization will be disabled.
   * @return The modified {@link LinearSystem.Builder} object for method chaining.
   */
  public Pivot enableVisualization(Transform3d offset) {
    this.pivotOffset = offset;
    return this;
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();

    if (pivotOffset != null) {
      // todo
    }
  }

  public void runPosition(int slot, Angle angle) {
    if (tempCritical) return;

    io.setPosition(slot, angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", angle.in(Degrees));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runPosition(Angle angle) {
    runPosition(0, angle);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(inputs.velocityDegPerSec);
  }

  public double getVelocityDegPerSec() {
    return inputs.velocityDegPerSec;
  }
}
