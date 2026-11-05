package org.saxonrobotics.saxonutil.lib.io.motors.elevator;

import static org.wpilib.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.saxonrobotics.saxonutil.lib.io.motors.Motor;
import org.saxonrobotics.saxonutil.lib.io.motors.MotorIO;
import org.saxonrobotics.saxonutil.lib.io.sensors.encoder.EncoderIO;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public class LinearSystem extends Motor<LinearSystemIO, LinearSystemIOInputsAutoLogged> {
  private final Function<Distance, Angle> distanceToAngle;
  private final Function<Angle, Distance> angleToDistance;
  private final Pose3d[] visualizedComponents;

  private LinearSystem(
      String name,
      LinearSystemIO io,
      EncoderIO encoderIO,
      BooleanSupplier brakeMode,
      Function<Distance, Angle> distanceToAngle,
      Function<Angle, Distance> angleToDistance,
      int numStages) {
    super(name, io, new LinearSystemIOInputsAutoLogged(), encoderIO, brakeMode);
    io.configure(true, true);
    this.distanceToAngle = distanceToAngle;
    this.angleToDistance = angleToDistance;
    if (numStages <= 0) {
      this.visualizedComponents = null;
    } else {
      this.visualizedComponents = new Pose3d[numStages];
    }
  }

  /** {@inheritDoc} */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();

    if (visualizedComponents != null) {
      double carriagePosition = angleToDistance.apply(getPosition()).in(Meters);
      for (int i = 0; i < visualizedComponents.length - 1; i++) {
        double stageHeight = carriagePosition * (i + 1) / visualizedComponents.length;
        visualizedComponents[i] = new Pose3d(0, 0, stageHeight, Rotation3d.kZero);
      }
      visualizedComponents[visualizedComponents.length - 1] =
          new Pose3d(0, 0, carriagePosition, Rotation3d.kZero);
      Logger.recordOutput(name + "/RobotComponents", visualizedComponents);
    }
  }

  public void runPosition(Angle angle) {
    if (tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointRad", angle.in(Radians));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runPosition(Distance position) {
    if (distanceToAngle == null) {
      throw new IllegalStateException(
          "Drum radius or custom position function must be set in the builder in order to use linear position requests!");
    }
    runPosition(distanceToAngle.apply(position));
  }

  public void runVelocity(double rps) {
    if (tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Radians.of(inputs.positionRad);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public Distance getHeight() {
    return angleToDistance.apply(getPosition());
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public static class Builder {
    private final String name;
    private final LinearSystemIO io;
    private EncoderIO encoderIO = inputs -> {};
    private BooleanSupplier brakeMode = () -> true;
    private Function<Distance, Angle> distanceToAngle;
    private Function<Angle, Distance> angleToDistance;
    private int numStages = 1;
    private boolean visualized = false;

    /** Create a builder that constructs a {@link LinearSystem}. */
    public Builder(String name, LinearSystemIO io) {
      this.name = name;
      this.io = io;
    }

    /** Optionally add an external encoder to use with this elevator. */
    public Builder addEncoder(EncoderIO encoderIO) {
      this.encoderIO = encoderIO;
      return this;
    }

    /** Set a custom supplier to determine the neutral output of the motor. */
    public Builder setBrakeMode(BooleanSupplier brakeMode) {
      this.brakeMode = brakeMode;
      return this;
    }

    /**
     * Set the number of stages in this elevator. This is important for visualization and correctly
     * converting between rotational and linear motion.
     */
    public Builder setNumStages(int numStages) {
      this.numStages = numStages;
      return this;
    }

    /**
     * Set the drum radius of this elevator. This is typically the radius of the sprocket, and is
     * used to convert between rotational and linear motion. In order for the resulting functions to
     * be accurate, the number of stages must be set using {@link setNumStages(int)}.
     */
    public Builder setDrumRadius(double drumRadiusMeters) {
      // don't override if manually set
      if (distanceToAngle == null) {
        distanceToAngle =
            distance -> Radians.of(distance.in(Meters) / drumRadiusMeters / numStages);
      }
      if (angleToDistance == null) {
        angleToDistance = angle -> Meters.of(numStages * angle.in(Radians) * drumRadiusMeters);
      }
      return this;
    }

    /** Use custom user-provided functions to convert between rotational and linear motion. */
    public Builder useCustomPositionFunctions(
        Function<Distance, Angle> distanceToAngle, Function<Angle, Distance> angleToDistance) {
      this.distanceToAngle = distanceToAngle;
      this.angleToDistance = angleToDistance;
      return this;
    }

    /**
     * Automatically set up visualization for this mechanism. The number of stages and drum radius
     * must be set in order for this method to work.
     */
    public Builder enableVisualization() {
      if (angleToDistance == null) {
        throw new IllegalStateException(
            "Drum radius or custom position functions must be set in the builder in order to visualize the mechanism!");
      }
      visualized = true;
      return this;
    }

    /**
     * Construct a {@link LinearSystem} object using the parameters given to the builder. This is
     * the final method call in the builder chain.
     */
    public LinearSystem build() {
      if (!visualized) {
        numStages = 0;
      }
      return new LinearSystem(
          name, io, encoderIO, brakeMode, distanceToAngle, angleToDistance, numStages);
    }
  }
}
