package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          50,
          SingleJointedArmSim.estimateMOI(5, Units.lbsToKilograms(1)),
          Units.inchesToMeters(5),
          -2 * Math.PI,
          2 * Math.PI,
          true,
          0,
          0,
          0);

  @Override
  public ShooterState.InputState getPosition() {
    armSim.update(0.02); // Update simulation with 20ms timestep
    return new ShooterState.InputState(
        Units.radiansToDegrees(armSim.getAngleRads()),
        Units.radiansToDegrees(armSim.getVelocityRadPerSec()));
  }

  @Override
  public void setPosition(ShooterState.OutputState output) {
    output
        .voltage()
        .ifPresent(
            (volts) -> {
              armSim.setInputVoltage(volts);
            });
  }
}
