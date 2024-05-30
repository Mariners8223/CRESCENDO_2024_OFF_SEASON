package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.function.Supplier;
import kotlin.Metadata;
import kotlin.jvm.internal.Intrinsics;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

@Metadata(
        mv = {1, 9, 0},
        k = 1,
        xi = 48,
        d1 = {"\u0000J\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0010\u0006\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0002\b\u000e\n\u0002\u0010\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\u0018\u00002\u00020\u0001B!\u0012\f\u0010\u0002\u001a\b\u0012\u0004\u0012\u00020\u00040\u0003\u0012\f\u0010\u0005\u001a\b\u0012\u0004\u0012\u00020\u00060\u0003¢\u0006\u0002\u0010\u0007J\b\u0010\u0015\u001a\u00020\tH\u0016J\b\u0010\u0016\u001a\u00020\tH\u0016J\b\u0010\u0017\u001a\u00020\tH\u0016J\b\u0010\u0018\u001a\u00020\tH\u0016J\b\u0010\u0019\u001a\u00020\tH\u0016J\b\u0010\u001a\u001a\u00020\fH\u0016J\b\u0010\u001b\u001a\u00020\tH\u0016J\b\u0010\u001c\u001a\u00020\tH\u0016J\b\u0010\u001d\u001a\u00020\tH\u0016J\u0012\u0010\u001e\u001a\u00020\u001f2\b\u0010 \u001a\u0004\u0018\u00010!H\u0016J\u0010\u0010\"\u001a\u00020\u001f2\u0006\u0010#\u001a\u00020$H\u0016J\b\u0010%\u001a\u00020\u001fH\u0016R\u000e\u0010\b\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\n\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u000b\u001a\u00020\fX\u0082\u000e¢\u0006\u0002\n\u0000R\u0014\u0010\u0005\u001a\b\u0012\u0004\u0012\u00020\u00060\u0003X\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\r\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u000e\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u000f\u001a\u00020\u0010X\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\u0011\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0012\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u0014\u0010\u0002\u001a\b\u0012\u0004\u0012\u00020\u00040\u0003X\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\u0013\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0014\u001a\u00020\tX\u0082\u000e¢\u0006\u0002\n\u0000¨\u0006&"},
        d2 = {"Lfrc/util/FastGyros/SimGyroIO;", "Lfrc/util/FastGyros/GyroIO;", "twistSupplier", "Ljava/util/function/Supplier;", "Ledu/wpi/first/math/geometry/Twist2d;", "chassisSpeedsSupplier", "Ledu/wpi/first/math/kinematics/ChassisSpeeds;", "(Ljava/util/function/Supplier;Ljava/util/function/Supplier;)V", "accelerationX", "", "accelerationY", "angle", "Ledu/wpi/first/math/geometry/Rotation2d;", "displacementX", "displacementY", "lock", "Ljava/util/concurrent/locks/ReentrantLock;", "prevVelocityX", "prevVelocityY", "velocityX", "velocityY", "getAccelerationX", "getAccelerationY", "getAngleDegrees", "getPitch", "getRoll", "getRotation2d", "getVelocityX", "getVelocityY", "getYaw", "initSendable", "", "builder", "Ledu/wpi/first/util/sendable/SendableBuilder;", "reset", "newPose", "Ledu/wpi/first/math/geometry/Pose2d;", "update", "BasicSwerve"}
)
public final class SimGyroIO implements GyroIO {
    private final Supplier<Twist2d> twistSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Rotation2d angle;
    private double velocityX;
    private double velocityY;
    private double displacementX;
    private double displacementY;
    private double accelerationX;
    private double accelerationY;
    private double prevVelocityX;
    private double prevVelocityY;

    public SimGyroIO(Supplier<Twist2d> twistSupplier,Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        Intrinsics.checkNotNullParameter(twistSupplier, "twistSupplier");
        Intrinsics.checkNotNullParameter(chassisSpeedsSupplier, "chassisSpeedsSupplier");
        this.twistSupplier = twistSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.angle = new Rotation2d();
    }

    public double getAngleDegrees() {
        double var1;
            var1 = this.angle.getDegrees();

        return var1;
    }

    @NotNull
    public Rotation2d getRotation2d() {
        Rotation2d var1;
            var1 = this.angle;

        return var1;
    }

    public void update() {
            Object var2 = this.twistSupplier.get();
            Intrinsics.checkNotNullExpressionValue(var2, "get(...)");
            Twist2d twist = (Twist2d)var2;
            Rotation2d var6 = Rotation2d.fromRadians(twist.dtheta + this.angle.getRadians());
            Intrinsics.checkNotNullExpressionValue(var6, "fromRadians(...)");
            this.angle = var6;
            Rotation2d[] var7 = new Rotation2d[]{this.angle};
            Logger.recordOutput("Gyro/angle", (StructSerializable[])var7);
            this.displacementX += twist.dx * this.angle.getCos() - twist.dy * this.angle.getSin();
            this.displacementY += twist.dy * this.angle.getCos() + twist.dx * this.angle.getSin();
            Pose2d[] var8 = new Pose2d[]{new Pose2d(this.displacementX, this.displacementY, this.angle)};
            Logger.recordOutput("Gyro/EstimatedPose", (StructSerializable[])var8);
            Object var3 = this.chassisSpeedsSupplier.get();
            Intrinsics.checkNotNullExpressionValue(var3, "get(...)");
            ChassisSpeeds chassisSpeeds = (ChassisSpeeds)var3;
            this.velocityX = chassisSpeeds.vxMetersPerSecond;
            this.velocityY = chassisSpeeds.vyMetersPerSecond;
            this.accelerationX = (this.velocityX - this.prevVelocityX) / 0.01;
            this.accelerationY = (this.velocityY - this.prevVelocityY) / 0.01;
            Logger.recordOutput("Gyro/PrevVelocityX", this.prevVelocityX);
            Logger.recordOutput("Gyro/PrevVelocityY", this.prevVelocityY);
            this.prevVelocityX = this.velocityX;
            this.prevVelocityY = this.velocityY;
            Logger.recordOutput("Gyro/velocityX", this.velocityX);
            Logger.recordOutput("Gyro/velocityY", this.velocityY);
            Logger.recordOutput("Gyro/accelerationX", this.accelerationX);
            Logger.recordOutput("Gyro/accelerationY", this.accelerationY);

    }

    public double getYaw() {
        double var1;
            var1 = -this.angle.getDegrees();

        return var1;
    }

    public double getPitch() {
        return 0.0;
    }

    public double getRoll() {
        return 0.0;
    }

    public double getVelocityX() {
        double var1;
            var1 = this.velocityX;

        return var1;
    }

    public double getVelocityY() {
        double var1;
            var1 = this.velocityY;
        return var1;
    }

    public double getAccelerationX() {
        double var1;
            var1 = this.accelerationX;

        return var1;
    }

    public double getAccelerationY() {
        double var1;
            var1 = this.accelerationY;

        return var1;
    }

    public void reset(@NotNull Pose2d newPose) {
        Intrinsics.checkNotNullParameter(newPose, "newPose");
            Rotation2d var2 = newPose.getRotation();
            Intrinsics.checkNotNullExpressionValue(var2, "getRotation(...)");
            this.angle = var2;
            this.displacementX = newPose.getTranslation().getX();
            this.displacementY = newPose.getTranslation().getY();

    }

    public void initSendable(@Nullable SendableBuilder builder) {
        if (builder != null) {
            builder.addDoubleProperty("angle", this::getAngleDegrees, null);
        }

    }
}
