package frc.robot.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterState;
import org.littletonrobotics.junction.mechanism.*;

public class Mech2DSim {
  private final Mechanism2d panel;
  private final MechanismRoot2d root;
  private final MechanismLigament2d arm;

  private final Shooter shooter;

  public Mech2DSim(Shooter shooter) {
    this.shooter = shooter;
    this.panel =
        new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100)); // view panel size
    this.root = this.panel.getRoot("root", Units.inchesToMeters(10), Units.inchesToMeters(10));
    this.arm =
        this.root.append(
            new MechanismLigament2d("arm", Units.inchesToMeters(5), 0, 6, new Color8Bit()));
  }

  public void periodic() {
    ShooterState.InputState state = shooter.getCurrentState();
    if (state != null) {
      this.arm.setAngle(state.currentAngleeDeg());
    }
    // Logger.getInstance()recordOutput("Mech2DSim/Mech2d", panel);
    SmartDashboard.putData("Mech2d", panel);
  }
}
