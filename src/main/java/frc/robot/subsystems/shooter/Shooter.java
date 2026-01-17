package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  private final ProfiledPIDController controller;
  private final ArmFeedforward ff;

  private ShooterState.InputState currentState;

  private ShooterState.GoalState goal = new ShooterState.GoalState(45);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.controller = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0, 0));
    this.ff = new ArmFeedforward(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      currentState = this.io.getPosition();
      double effort = controller.calculate(currentState.currentAngleeDeg(), goal.positionDeg());
      double feedforward =
          this.ff.calculate(
              Units.degreesToRadians(currentState.currentAngleeDeg()),
              Units.degreesToRadians(currentState.currentVelocityDegPerSec()));

      effort += feedforward;

      this.io.setPosition(new ShooterState.OutputState(Optional.of(effort)));
    }
  }

  public ShooterState.InputState getCurrentState() {
    return this.currentState;
  }

  public Command setArmPosition(double positionDeg) {
    return new InstantCommand(
        () -> {
          this.goal = new ShooterState.GoalState(positionDeg);
        });
  }
}
