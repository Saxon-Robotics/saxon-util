package first.robot.util.io.motors.elevator;

import static org.wpilib.units.Units.Radians;

import first.robot.util.io.motors.MotorIOSim;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.units.measure.Angle;

public class LinearSystemIOSim extends MotorIOSim implements LinearSystemIO {
  private final ElevatorSim sim;
  private ProfiledPIDController profiledPID;
  private ElevatorFeedforward feedforward;

  private final double gearing;
  private final double drumRadiusMeters;

  private double targetPositionRad = 0.0;

  public LinearSystemIOSim(
      DCMotor gearbox,
      LinearMechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);

    this.gearing = constraints.reduction();
    this.drumRadiusMeters = constraints.drumRadiusMeters();

    this.sim =
        new ElevatorSim(
            gearbox,
            gearing,
            constraints.carriageMassKg(),
            drumRadiusMeters,
            constraints.minHeightMeters(),
            constraints.maxHeightMeters(),
            true,
            0.0);
  }

  /**
   * Set up the simulation to use profiled trapezoidal PID instead of standard PID
   *
   * @param maxVelocity Max velocity of the elevator, in radians per second.
   * @param maxAcceleration Max acceleration of the elevator, in radians per second squared.
   * @return The modified {@link LinearSystemIOSim} object for method chaining.
   */
  public LinearSystemIOSim withProfiledPID(double maxVelocity, double maxAcceleration) {
    profiledPID =
        new ProfiledPIDController(
            pid.getP(),
            pid.getI(),
            pid.getD(),
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    return this;
  }

  public LinearSystemIOSim withFeedforward(double kS, double kG, double kV, double kA) {
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    return this;
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    inputs.connected = true;
    if (isClosedLoop) {
      double currentMotorRad = carriageMetersToRad(sim.getPosition());
      if (profiledPID != null) {
        appliedVoltage =
            Math.clamp(
                profiledPID.calculate(currentMotorRad, targetPositionRad)
                    + (feedforward != null
                        ? feedforward.calculate(profiledPID.getSetpoint().velocity)
                        : 0),
                -12,
                12);
      } else {
        appliedVoltage = Math.clamp(pid.calculate(currentMotorRad, targetPositionRad), -12, 12);
      }
    }
    updateMotorInputs(inputs);

    sim.setInputVoltage(appliedVoltage);
    sim.update(0.020);
    inputs.positionRad = carriageMetersToRad(sim.getPosition());
    inputs.velocityRadPerSec = carriageMetersToRad(sim.getVelocity());
    inputs.statorCurrentAmps = sim.getCurrentDraw();
  }

  @Override
  public void setPosition(Angle angle) {
    this.targetPositionRad = angle.in(Radians);
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(Angle angle) {
    sim.setState(radsToCarriageMeters(angle.in(Radians)), 0.0);
  }

  private double carriageMetersToRad(double meters) {
    double drumRotations = meters / drumRadiusMeters;
    return drumRotations * gearing;
  }

  private double radsToCarriageMeters(double radians) {
    double drumRadians = radians / gearing;
    return drumRadians * drumRadiusMeters;
  }
}
