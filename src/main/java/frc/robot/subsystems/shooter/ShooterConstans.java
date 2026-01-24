package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstans {

  public static final double SHOOTER_PIVOT_MAX_ANGLE = 2;
  public static final double SHOOTER_PIVOT_MIN_ANGLE = 0;
  public static final double SHOOTER_PIVOT_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double SHOOTER_PIVOT_PID_VELOCITY_TOLERANCE = 0.5;
  public static final double[] SHOOTER_PIVOT_PID_REAL = {1, 0, 0};
  public static final double[] SHOOTER_PIVOT_FEEDFORWARD_REAL = {0.123, 0.6465, 0.22, 0.04};

  public static final double[] SHOOTER_PIVOT_PID_REAL_ACTIVE = {0, 0, 0};
  public static final double[] SHOOTER_PIVOT_FEEDFORWARD_REAL_ACTIVE = {0, 0, 0, 0};
  public static final double SHOOTER_PIVOT_MAX_VELOCITY = 1;
  public static final double SHOOTER_PIVOT_MAX_ACCELERATION = 1;
  public static final double SHOOTER_PIVOT_TOLERANCE = 1.0;
  public static int pivotMotorCurrentLimit = 50;
  public static double pivotKp;
  public static double pivotKd;
  public static double pivotKi;
  public static int pivotMotorID = 40;
  public static double minOutput;
  public static double maxOutput;
  public static double kPivotSpeed = 0.5;

  public static double armLengthMeters = Units.inchesToMeters(5);

  public static class ShooterSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0};
    public static final double[] kPivotSimFF = {0, 0.574, 0, 0};

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 1 / 4096;

    public static final double kArmReduction = 50;
    public static final double kArmMass = 1; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(5);
  }
}
