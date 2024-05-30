package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import kotlin.Metadata;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@Metadata(
        mv = {1, 9, 0},
        k = 1,
        xi = 48,
        d1 = {"\u00000\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0006\n\u0002\b\u0005\n\u0002\u0018\u0002\n\u0002\b\u0004\n\u0002\u0010\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\bf\u0018\u00002\u00020\u0001J\b\u0010\u0002\u001a\u00020\u0003H&J\b\u0010\u0004\u001a\u00020\u0003H&J\b\u0010\u0005\u001a\u00020\u0003H&J\b\u0010\u0006\u001a\u00020\u0003H&J\b\u0010\u0007\u001a\u00020\u0003H&J\b\u0010\b\u001a\u00020\tH&J\b\u0010\n\u001a\u00020\u0003H&J\b\u0010\u000b\u001a\u00020\u0003H&J\b\u0010\f\u001a\u00020\u0003H&J\u0012\u0010\r\u001a\u00020\u000e2\b\u0010\u000f\u001a\u0004\u0018\u00010\u0010H&J\u0012\u0010\u0011\u001a\u00020\u000e2\b\b\u0002\u0010\u0012\u001a\u00020\u0013H&J\b\u0010\u0014\u001a\u00020\u000eH&¨\u0006\u0015"},
        d2 = {"Lfrc/util/FastGyros/GyroIO;", "Ledu/wpi/first/util/sendable/Sendable;", "getAccelerationX", "", "getAccelerationY", "getAngleDegrees", "getPitch", "getRoll", "getRotation2d", "Ledu/wpi/first/math/geometry/Rotation2d;", "getVelocityX", "getVelocityY", "getYaw", "initSendable", "", "builder", "Ledu/wpi/first/util/sendable/SendableBuilder;", "reset", "newPose", "Ledu/wpi/first/math/geometry/Pose2d;", "update", "BasicSwerve"}
)
public interface GyroIO extends Sendable {
    double getAngleDegrees();

    @NotNull
    Rotation2d getRotation2d();

    void update();

    double getYaw();

    double getPitch();

    double getRoll();

    double getVelocityX();

    double getVelocityY();

    double getAccelerationX();

    double getAccelerationY();

    void reset(@NotNull Pose2d var1);

    void initSendable(@Nullable SendableBuilder var1);

    @Metadata(
            mv = {1, 9, 0},
            k = 3,
            xi = 48
    )
    public static final class DefaultImpls {
        // $FF: synthetic method
        public static void reset$default(GyroIO var0, Pose2d var1, int var2, Object var3) {
            if (var3 != null) {
                throw new UnsupportedOperationException("Super calls with default arguments not supported in this target, function: reset");
            } else {
                if ((var2 & 1) != 0) {
                    var1 = new Pose2d();
                }

                var0.reset(var1);
            }
        }
    }
}
