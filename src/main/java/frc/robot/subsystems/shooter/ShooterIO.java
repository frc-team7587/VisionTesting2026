package frc.robot.subsystems.shooter;

public interface ShooterIO {
  public ShooterState.InputState getPosition();

  public void setPosition(ShooterState.OutputState output);
}
