package org.saxonrobotics.saxonutil.lib.subsystems;

import java.util.HashSet;
import java.util.Set;
import lombok.Getter;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

public class SubsystemManager {
  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = MatchState.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.RED;
  }

  private static SubsystemManager instance;

  /** Returns the singleton instance of the {@link SubsystemManager}. */
  public static SubsystemManager getInstance() {
    if (instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }

  @Getter private boolean robotEnabled;

  private final Set<Subsystem> subsystems = new HashSet<>();

  public void registerSubsystem(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem == null || this.subsystems.contains(subsystem)) {
        continue;
      }
      this.subsystems.add(subsystem);
    }
  }

  /**
   * Updates the {@link SubsystemManager#robotEnabled} flag to enabled and runs all subsystem enable
   * logic. Call this in your robot's {@code disabledExit()} method.
   */
  public void enable() {
    robotEnabled = true;
    subsystems.forEach(Subsystem::enable);
  }

  /**
   * Updates the {@link SubsystemManager#robotEnabled} flag to disabled and runs all subsystem
   * disable logic. Call this in your robot's {@code disabledInit()} method.
   */
  public void disable() {
    robotEnabled = false;
    subsystems.forEach(Subsystem::disable);
  }
}
