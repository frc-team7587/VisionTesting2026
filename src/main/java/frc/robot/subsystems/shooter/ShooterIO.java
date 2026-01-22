package frc.robot.subsystems.shooter;

public interface ShooterIO {
  //public ShooterState.InputState getPosition();

      /**
     * Sets the speed of the pivot motor.
     * @param speed The speed to set the pivot motor to.
     */
    public void setPivotSpeed(double speed);

    /**
     * Sets the position of the pivot motor.
     * @param position The position to set the pivot motor to.
     */
    public void setPivotPosition(double position);

    /**
     * Gets the encoder value of the pivot motor.
     * @return The encoder value of the pivot motor.
     */
    public double getPivotPosition();

    /**
     * Resets the encoder value of the pivot motor.
     */
    public void reset();
}
