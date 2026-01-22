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
  public void setPivotSpeed(double speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPivotSpeed'");
  }

  @Override
  public void setPivotPosition(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPivotPosition'");
  }

  @Override
  public double getPivotPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotPosition'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }
}
