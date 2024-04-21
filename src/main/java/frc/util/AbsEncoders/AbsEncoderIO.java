package frc.util.AbsEncoders;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AbsEncoderIO {
  /**
   * gets the current angle of the encoder (not bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  public double getPosition();

  /**
   * gets the current angle of the encoder (bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  public double getAbsolutePosition();

  /**
   * gets the current angle of the encoder (not bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  public Rotation2d getRotation2d();
}
