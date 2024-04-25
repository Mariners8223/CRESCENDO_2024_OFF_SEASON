package frc.util.AbsEncoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ThroughBoreRIOIO implements AbsEncoderIO{
  private final DutyCycleEncoder encoder;

  public ThroughBoreRIOIO(int channel, double conversionFactor, double zeroOffset){
    encoder = new DutyCycleEncoder(channel);

    encoder.setDistancePerRotation(conversionFactor);

    encoder.setPositionOffset(zeroOffset);
  }

  @Override
  public double getPosition() {
    return encoder.getDistance();
  }

  @Override
  public double getAbsolutePosition() {
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(getPosition());
  }
}
