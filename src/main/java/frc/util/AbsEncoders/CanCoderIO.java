package frc.util.AbsEncoders;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class CanCoderIO implements AbsEncoderIO{
  private final CANcoder canCoder;
  private final double conversionFactor;

  /**
   * Creates a new CanCoderIO
   * @param canCoderID the ID of the CanCoder
   * @param isInverted if the CanCoder is inverted
   * @param conversionFactor the conversion factor to convert the canCoders rotations to a different unit
   * @param zeroOffset the offset of the CanCoder
   * @param isFullCircle if the CanCoder is a full circle (if the reading is 0-1 or -0.5-0.5)
   * @param updateRate the update rate of the CanCoder in Hz (default 50 (change only if using threads))
   */
  public CanCoderIO(int canCoderID, boolean isInverted, double conversionFactor, double zeroOffset, boolean isFullCircle, double updateRate) {
    this.canCoder = new CANcoder(canCoderID);

    CANcoderConfiguration config = new CANcoderConfiguration();

    config.FutureProofConfigs = false;

    config.MagnetSensor.SensorDirection = isInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

    config.MagnetSensor.AbsoluteSensorRange = isFullCircle ? AbsoluteSensorRangeValue.Unsigned_0To1 : AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    config.MagnetSensor.MagnetOffset = zeroOffset;

    canCoder.getConfigurator().apply(config);

    this.conversionFactor = conversionFactor;

    canCoder.getPosition().setUpdateFrequency(updateRate);
    canCoder.getAbsolutePosition().setUpdateFrequency(updateRate);

    canCoder.optimizeBusUtilization();
  }

  /**
   * Creates a new CanCoderIO
   * @param canCoderID the ID of the CanCoder
   * @param isInverted if the CanCoder is inverted
   * @param conversionFactor the conversion factor to convert the canCoders rotations to a different unit
   * @param zeroOffset the offset of the CanCoder
   * @param isFullCircle if the CanCoder is a full circle (if the reading is 0-1 or -0.5-0.5)
   */
  public CanCoderIO(int canCoderID, boolean isInverted, double conversionFactor, double zeroOffset, boolean isFullCircle){
    this(canCoderID, isInverted, conversionFactor, zeroOffset, isFullCircle, 50);
  }

  /**
   * gets the current angle of the encoder (not bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  @Override
  public double getPosition() {
    return canCoder.getPosition().getValueAsDouble() * conversionFactor;
  }

  /**
   * gets the current angle of the encoder (bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  @Override
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * conversionFactor;
  }

  /**
   * gets the current angle of the encoder (not bound in 0 - 1 or -0.5 - 0.5)
   * @return the rotations of the encoder times the conversion factor (default is rotations)
   */
  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(getPosition());
  }
}
