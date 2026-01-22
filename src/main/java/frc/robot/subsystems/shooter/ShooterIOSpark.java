package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.shooter.ShooterState.InputState;
import frc.robot.subsystems.shooter.ShooterState.OutputState;

public class ShooterIOSpark implements ShooterIO {
  // hardware
  private final SparkMax pivotSpark;
  private final RelativeEncoder pivotEncoder;
  // Closed loop controllers
  private final SparkClosedLoopController pivotController;

  public ShooterIOSpark() {
    this.pivotSpark = new SparkMax(ShooterConstans.pivotMotorID, SparkMax.MotorType.kBrushless);
    this.pivotEncoder = pivotSpark.getEncoder();
    this.pivotController = pivotSpark.getClosedLoopController();

    // Configure pivot motor
    var pivotConfig = new SparkMaxConfig();
    var pivotSoftLimitConfig = new SoftLimitConfig();
    pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ShooterConstans.pivotMotorCurrentLimit);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstans.pivotKp, 0.0, ShooterConstans.pivotKd)
        .outputRange(ShooterConstans.minOutput, ShooterConstans.maxOutput);
    pivotSoftLimitConfig
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(180.0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0.0);
    pivotSpark.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  @Override
  public void setPivotSpeed(double speed){ 
      pivotSpark.set(speed);
  }
  @Override
  public void setPivotPosition(double position){
      pivotController.setReference(position, ControlType.kPosition);
  }
  @Override
  public double getPivotPosition(){
      return pivotEncoder.getPosition();
  }
  @Override
  public void reset(){
      pivotEncoder.setPosition(0);
  }
}
