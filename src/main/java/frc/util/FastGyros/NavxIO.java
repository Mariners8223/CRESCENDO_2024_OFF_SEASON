// NavxIO.java
package frc.util.FastGyros;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import kotlin.Metadata;
import kotlin.jvm.internal.Intrinsics;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@Metadata(
        mv = {1, 9, 0},
        k = 1,
        xi = 48,
        d1 = {"\u0000J\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u000b\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0006\n\u0002\b\u0005\n\u0002\u0018\u0002\n\u0002\b\u0004\n\u0002\u0010\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\u0018\u00002\u00020\u0001B\r\u0012\u0006\u0010\u0002\u001a\u00020\u0003¢\u0006\u0002\u0010\u0004J\b\u0010\u000b\u001a\u00020\fH\u0016J\b\u0010\r\u001a\u00020\fH\u0016J\b\u0010\u000e\u001a\u00020\fH\u0016J\b\u0010\u000f\u001a\u00020\fH\u0016J\b\u0010\u0010\u001a\u00020\fH\u0016J\b\u0010\u0011\u001a\u00020\u0012H\u0016J\b\u0010\u0013\u001a\u00020\fH\u0016J\b\u0010\u0014\u001a\u00020\fH\u0016J\b\u0010\u0015\u001a\u00020\fH\u0016J\u0012\u0010\u0016\u001a\u00020\u00172\b\u0010\u0018\u001a\u0004\u0018\u00010\u0019H\u0016J\u0010\u0010\u001a\u001a\u00020\u00172\u0006\u0010\u001b\u001a\u00020\u001cH\u0016J\b\u0010\u001d\u001a\u00020\u0017H\u0016R\u000e\u0010\u0005\u001a\u00020\u0006X\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\u0002\u001a\u00020\u0003X\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\u0007\u001a\u00020\bX\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\t\u001a\u00020\nX\u0082\u0004¢\u0006\u0002\n\u0000¨\u0006\u001e"},
        d2 = {"Lfrc/util/FastGyros/NavxIO;", "Lfrc/util/FastGyros/GyroIO;", "isInverted", "", "(Z)V", "inputs", "Lfrc/util/FastGyros/NavxInputs;", "lock", "Ljava/util/concurrent/locks/ReentrantLock;", "navx", "Lcom/kauailabs/navx/frc/AHRS;", "getAccelerationX", "", "getAccelerationY", "getAngleDegrees", "getPitch", "getRoll", "getRotation2d", "Ledu/wpi/first/math/geometry/Rotation2d;", "getVelocityX", "getVelocityY", "getYaw", "initSendable", "", "builder", "Ledu/wpi/first/util/sendable/SendableBuilder;", "reset", "newPose", "Ledu/wpi/first/math/geometry/Pose2d;", "update", "BasicSwerve"}
)
public final class NavxIO implements GyroIO {
    @AutoLog
    public static class NavxInputs{
        public Rotation2d angle = new Rotation2d();
        public Rotation2d rotationOffset = new Rotation2d();
        public double velocityX;
        public double velocityY;
        public double accelerationX;
        public double accelerationY;
        public Pose2d estimatedPose = new Pose2d();
        public double prevVelocityX;
        public double prevVelocityY;
        public long prevTimeStamp;
        public double yaw;
        public double pitch;
        public double roll;
    }


    private final boolean isInverted;
    private final AHRS navx;
    private final NavxInputsAutoLogged inputs;

    public NavxIO(boolean isInverted) {
        this.isInverted = isInverted;
        this.navx = new AHRS();
        this.inputs = new NavxInputsAutoLogged();
   }

    public void reset(Pose2d newPose) {
        Intrinsics.checkNotNullParameter(newPose, "newPose");

        this.navx.reset();
        this.inputs.angle = newPose.getRotation();
        this.inputs.rotationOffset = newPose.getRotation().unaryMinus();
        this.inputs.estimatedPose = newPose;

    }

    public double getAngleDegrees() {
        double var1;
        var1 = this.inputs.angle.getDegrees();

        return var1;
    }

    @NotNull
    public Rotation2d getRotation2d() {
        Rotation2d var1;
            var1 = this.inputs.angle;

        return var1;
    }

    public void update() {
            // inputs.angle = if(isInverted) Rotation2d.fromDegrees(navx.angle - inputs.rotationOffset.degrees) else Rotation2d.fromDegrees(-navx.angle - inputs.rotationOffset.degrees);
            inputs.angle = isInverted ? Rotation2d.fromDegrees(navx.getAngle() - inputs.rotationOffset.getDegrees()) : Rotation2d.fromDegrees(-navx.getAngle() - inputs.rotationOffset.getDegrees());
      
            inputs.yaw = navx.getYaw();
            inputs.pitch = navx.getPitch();
            inputs.roll = navx.getRoll();
      
            inputs.velocityX = inputs.angle.getCos() * navx.getVelocityX() - inputs.angle.getSin() * navx.getVelocityY();
            inputs.velocityY = inputs.angle.getSin() * navx.getVelocityX() - inputs.angle.getCos() * navx.getVelocityY();
      
            inputs.accelerationX = (inputs.angle.getCos() * navx.getWorldLinearAccelX() - inputs.angle.getSin() * navx.getWorldLinearAccelY()) * 9.18;
            inputs.accelerationY = (inputs.angle.getSin() * navx.getWorldLinearAccelX() - inputs.angle.getCos() * navx.getWorldLinearAccelY()) * 9.18;
      
            inputs.estimatedPose = new Pose2d((inputs.accelerationX + inputs.prevVelocityX) * (navx.getLastSensorTimestamp() - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.getX(),
              (inputs.accelerationY + inputs.prevVelocityY) * (navx.getLastSensorTimestamp() - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.getY(),    
              inputs.angle);
      
            inputs.prevVelocityX = inputs.velocityX;
            inputs.prevVelocityY = inputs.velocityY;
            inputs.prevTimeStamp = navx.getLastSensorTimestamp();

        Logger.processInputs("FastNavx", (LoggableInputs)this.inputs);
    }

    public double getYaw() {
        double var1;
            var1 = this.inputs.yaw;

        return var1;
    }

    public double getPitch() {
        double var1;
            var1 = this.inputs.pitch;

        return var1;
    }

    public double getRoll() {
        double var1;
            var1 = this.inputs.roll;

        return var1;
    }

    public double getVelocityX() {
        double var1;
            var1 = this.inputs.velocityX;

        return var1;
    }

    public double getVelocityY() {
        double var1;
            var1 = this.inputs.velocityY;

        return var1;
    }

    public double getAccelerationX() {
        double var1;
            var1 = this.inputs.accelerationX;

        return var1;
    }

    public double getAccelerationY() {
        double var1;
            var1 = this.inputs.accelerationY;

        return var1;
    }

    public void initSendable(@Nullable SendableBuilder builder) {
        this.navx.initSendable(builder);
    }
}