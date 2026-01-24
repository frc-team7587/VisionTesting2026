package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSpark implements ShooterIO {
  // hardware
  private final SparkMax pivotSpark;

  private final SparkMaxConfig pivotConfig;
  private final RelativeEncoder pivotEncoder;
  private final ProfiledPIDController pidController;
  private final ProfiledPIDController pidControllerActive;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private ArmFeedforward feedforwardActive = new ArmFeedforward(0, 0, 0, 0);
  // private final SparkClosedLoopController pivotController;

  private double setpoint = 0;

  private double kP = ShooterConstans.SHOOTER_PIVOT_PID_REAL[0],
      kI = ShooterConstans.SHOOTER_PIVOT_PID_REAL[1],
      kD = ShooterConstans.SHOOTER_PIVOT_PID_REAL[2];
  private double kActiveP = ShooterConstans.SHOOTER_PIVOT_PID_REAL_ACTIVE[0],
      kActiveI = ShooterConstans.SHOOTER_PIVOT_PID_REAL_ACTIVE[1],
      kActiveD = ShooterConstans.SHOOTER_PIVOT_PID_REAL_ACTIVE[2];

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / change in time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public ShooterIOSpark() {
    this.pivotSpark = new SparkMax(ShooterConstans.pivotMotorID, SparkMax.MotorType.kBrushless);
    // Configure pivot motor
    pivotConfig = new SparkMaxConfig();

    pivotConfig
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(ShooterConstans.pivotMotorCurrentLimit);

    pivotEncoder = pivotSpark.getEncoder();
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    pidController =
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                ShooterConstans.SHOOTER_PIVOT_MAX_VELOCITY,
                ShooterConstans.SHOOTER_PIVOT_MAX_ACCELERATION));

    pidControllerActive =
        new ProfiledPIDController(
            kActiveP,
            kActiveI,
            kActiveD,
            new TrapezoidProfile.Constraints(
                ShooterConstans.SHOOTER_PIVOT_MAX_VELOCITY,
                ShooterConstans.SHOOTER_PIVOT_MAX_ACCELERATION));
    pivotSpark.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configureFeedForward();
    configureFeedForwardActive();
  }

  private void configureFeedForward() {
    setkS(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL[3]);
  }

  private void configureFeedForwardActive() {
    setActivekS(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL_ACTIVE[0]);
    setActivekG(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL_ACTIVE[1]);
    setActivekV(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL_ACTIVE[2]);
    setActivekA(ShooterConstans.SHOOTER_PIVOT_FEEDFORWARD_REAL_ACTIVE[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = pivotEncoder.getVelocity();
    inputs.appliedVolts = pivotSpark.getAppliedOutput() * pivotSpark.getBusVoltage();
    inputs.setpointAngleRads = pidController.getSetpoint().position;

    inputs.currentAmps = new double[] {pivotSpark.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotSpark.getMotorTemperature()};
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("AlgaePivot/Desired Voltage", motorVolts);
    pivotSpark.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return pivotEncoder.getPosition();
  }

  @Override
  public double getAngVelocity() {
    return pivotEncoder.getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    pidController.reset(getAngle(), getAngVelocity());
    pidControllerActive.setGoal(setpoint);
    pidControllerActive.reset(getAngle(), getAngVelocity());
    Logger.recordOutput("AlgaePivot/Actual Setpoint", pidController.getSetpoint().position);
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = 0, ffOutput = 0;

    pidOutput = pidController.calculate(getAngle());

    // change in velocity / change in time = acceleration
    // Acceleration is used to calculate feedforward
    double acceleration =
        (pidController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    Logger.recordOutput("CoralPivot/Acceleration", acceleration);

    ffOutput =
        feedforward.calculate(
            pidController.getSetpoint().position,
            pidController.getSetpoint().velocity,
            acceleration);

    lastSpeed = pidController.getSetpoint().velocity;

    Logger.recordOutput("AlgaePivot/PID output", pidOutput);
    Logger.recordOutput("AlgaePivot/FF output", ffOutput);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotSpark.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < ShooterConstans.SHOOTER_PIVOT_PID_TOLERANCE;
  }

  @Override
  public void setP(double p) {
    pidController.setP(p);
  }

  @Override
  public void setI(double i) {
    pidController.setI(i);
  }

  @Override
  public void setD(double d) {
    pidController.setD(d);
  }

  @Override
  public void setkS(double kS) {
    feedforward.setKs(kS);
  }

  @Override
  public void setkG(double kG) {
    feedforward.setKg(kG);
  }

  @Override
  public void setkV(double kV) {
    feedforward.setKv(kV);
  }

  @Override
  public void setkA(double kA) {
    feedforward.setKa(kA);
  }

  @Override
  public double getkS() {
    return feedforward.getKs();
  }

  @Override
  public double getkG() {
    return feedforward.getKg();
  }

  @Override
  public double getkV() {
    return feedforward.getKv();
  }

  @Override
  public double getkA() {
    return feedforward.getKa();
  }

  @Override
  public double getP() {
    return kP;
  }

  @Override
  public double getI() {
    return kI;
  }

  @Override
  public double getD() {
    return kD;
  }

  @Override
  public void setActiveP(double p) {
    pidControllerActive.setP(p);
  }

  @Override
  public void setActiveI(double i) {
    pidControllerActive.setI(i);
  }

  @Override
  public void setActiveD(double d) {
    pidControllerActive.setD(d);
  }

  @Override
  public void setActivekS(double kS) {
    feedforwardActive.setKs(kS);
  }

  @Override
  public void setActivekG(double kG) {
    feedforwardActive.setKg(kG);
  }

  @Override
  public void setActivekV(double kV) {
    feedforwardActive.setKv(kV);
  }

  @Override
  public void setActivekA(double kA) {
    feedforwardActive.setKa(kA);
  }

  @Override
  public double getActivekS() {
    return feedforwardActive.getKs();
  }

  @Override
  public double getActivekG() {
    return feedforwardActive.getKg();
  }

  @Override
  public double getActivekV() {
    return feedforwardActive.getKv();
  }

  @Override
  public double getActivekA() {
    return feedforwardActive.getKa();
  }

  @Override
  public double getActiveP() {
    return pidControllerActive.getP();
  }

  @Override
  public double getActiveI() {
    return pidControllerActive.getI();
  }

  @Override
  public double getActiveD() {
    return pidControllerActive.getD();
  }
}
