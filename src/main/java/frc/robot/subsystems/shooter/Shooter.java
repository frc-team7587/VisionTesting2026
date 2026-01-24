package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooter;

  private final ProfiledPIDController controller;
  private final ArmFeedforward ff;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private LoggedNetworkNumber logkS;
  private LoggedNetworkNumber logkG;
  private LoggedNetworkNumber logkV;
  private LoggedNetworkNumber logkA;

  private LoggedNetworkNumber logActiveP;
  private LoggedNetworkNumber logActiveI;
  private LoggedNetworkNumber logActiveD;

  private LoggedNetworkNumber logActivekS;
  private LoggedNetworkNumber logActivekG;
  private LoggedNetworkNumber logActivekV;
  private LoggedNetworkNumber logActivekA;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reall?ocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private double setpoint = 0;
  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  public static enum State {
    MANUAL,
    PID,
    SYSID
  }

  private State armState = State.MANUAL;
  private double manualSpeed = 0;

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
    this.controller = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0, 0));
    this.ff = new ArmFeedforward(0.0, 0.0, 0.0);

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/P", shooter.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/I", shooter.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/D", shooter.getD());

    logkS = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kS", shooter.getkS());
    logkG = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kG", shooter.getkG());
    logkV = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kV", shooter.getkV());
    logkA = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kA", shooter.getkA());

    logActiveP =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active P", shooter.getActiveP());
    logActiveI =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active I", shooter.getActiveI());
    logActiveD =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active D", shooter.getActiveD());

    logActivekS =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kS", shooter.getActivekS());
    logActivekG =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kG", shooter.getActivekG());
    logActivekV =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kV", shooter.getActivekV());
    logActivekA =
        new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kA", shooter.getActivekA());
  }

  @Override
  public void periodic() {
    shooter.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    ;

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Move arm based on state
    switch (armState) {
      case MANUAL:
        move(manualSpeed);
        break;
      case PID:
        runPID();
        break;
      default:
        break;
    }

    // Update the PID constants if they have changed
    if (logP.get() != shooter.getP()) shooter.setP(logP.get());

    if (logI.get() != shooter.getI()) shooter.setI(logI.get());

    if (logD.get() != shooter.getD()) shooter.setD(logD.get());
    if (logkS.get() != shooter.getkS()) shooter.setkS(logkS.get());

    if (logkG.get() != shooter.getkG()) shooter.setkG(logkG.get());

    if (logkV.get() != shooter.getkV()) shooter.setkV(logkV.get());

    if (logkA.get() != shooter.getkA()) shooter.setkA(logkA.get());

    if (logActiveP.get() != shooter.getActiveP()) shooter.setActiveP(logActiveP.get());
    if (logActiveI.get() != shooter.getActiveI()) shooter.setActiveI(logActiveI.get());

    if (logActiveD.get() != shooter.getActiveD()) shooter.setActiveD(logActiveD.get());

    if (logActivekS.get() != shooter.getActivekS()) shooter.setActivekS(logActivekS.get());

    if (logActivekG.get() != shooter.getActivekG()) shooter.setActivekG(logActivekG.get());

    if (logActivekV.get() != shooter.getActivekV()) shooter.setActivekV(logActivekV.get());
    if (logActivekA.get() != shooter.getActivekA()) shooter.setActivekA(logActivekA.get());

    // Log Inputs
    Logger.processInputs("AlgaePivot", inputs);

    // Logger.recordOutput(
    //     "AlgaePivot/PivotAbsoluteEncoderConnected",
    //     inputs.angleRads != AlgaePivotConstants.ALGAE_PIVOT_OFFSET);

  }

  @AutoLogOutput(key = "AlgaePivot/Is Voltage Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= ShooterConstans.SHOOTER_PIVOT_TOLERANCE;
  }

  public void move(double speed) {
    // limit the arm if its past the limit
    if (shooter.getAngle() > ShooterConstans.SHOOTER_PIVOT_MAX_ANGLE && speed > 0) {
      speed = 0;
    } else if (shooter.getAngle() < ShooterConstans.SHOOTER_PIVOT_MIN_ANGLE && speed < 0) {
      speed = 0;
    }

    shooter.setVoltage(speed * 12);

    isVoltageClose(speed * 12);
  }

  public void runPID() {
    if (setpoint > ShooterConstans.SHOOTER_PIVOT_MAX_ANGLE) {
      setpoint = ShooterConstans.SHOOTER_PIVOT_MAX_ANGLE;
    } else if (setpoint < ShooterConstans.SHOOTER_PIVOT_MIN_ANGLE) {
      setpoint = ShooterConstans.SHOOTER_PIVOT_MIN_ANGLE;
    }
    if ((shooter.getAngle() <= ShooterConstans.SHOOTER_PIVOT_MIN_ANGLE
            && shooter.getAngVelocity() < 0)
        || (shooter.getAngle() >= ShooterConstans.SHOOTER_PIVOT_MAX_ANGLE
            && shooter.getAngVelocity() > 0)) {
      shooter.setVoltage(0);
    } else {
      shooter.goToSetpoint();
    }
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    armState = State.PID;
    shooter.setSetpoint(setpoint);
    Logger.recordOutput("AlgaePivot/Setpoint", setpoint);
  }

  public void setManual(double speed) {
    manualSpeed = speed;
    if (speed != 0) {
      armState = State.MANUAL;
    }
  }

  public boolean atSetpoint() {
    return Math.abs(shooter.getAngle() - setpoint) < ShooterConstans.SHOOTER_PIVOT_PID_TOLERANCE
        && Math.abs(shooter.getAngVelocity())
            < ShooterConstans.SHOOTER_PIVOT_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return armMechanism.append(mechanism);
  }

  public MechanismLigament2d getArmMechanism() {
    armMechanism = new MechanismLigament2d("Algae Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));
    return armMechanism;
  }

  public Command PIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint), this)
        .andThen(new WaitUntilCommand(() -> atSetpoint()));
  }

  public Command InstantPIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint));
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new RunCommand(() -> setManual(speedSupplier.getAsDouble()), this)
        .finallyDo(
            () -> {
              manualSpeed = 0;
              move(0);
            });
  }

  public Command ManualCommand(double speed) {
    return ManualCommand(() -> speed);
  }
}
