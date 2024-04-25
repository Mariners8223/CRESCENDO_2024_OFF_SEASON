package frc.util.AbsEncoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.geometry.Rotation2d;


public class ThroughBoreSparkIO implements AbsEncoderIO{
  private final AbsoluteEncoder encoder;
  private final double conversionFactor;

  public ThroughBoreSparkIO(CANSparkFlex flex, double conversionFactor, double zeroOffset){
    encoder = flex.getAbsoluteEncoder();

    encoder.setPositionConversionFactor(conversionFactor);
    this.conversionFactor = conversionFactor;

    encoder.setZeroOffset(zeroOffset);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getAbsolutePosition() {
    return encoder.getPosition() % conversionFactor;
  }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(getPosition());
  }
}
