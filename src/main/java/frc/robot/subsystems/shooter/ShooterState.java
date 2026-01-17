package frc.robot.subsystems.shooter;

import java.util.Optional;

public class ShooterState {
  public record InputState(double currentAngleeDeg, double currentVelocityDegPerSec) {}

  public record OutputState(Optional<Double> voltage) {}

  public record GoalState(double positionDeg) {}
}
