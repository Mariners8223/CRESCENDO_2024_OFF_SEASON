package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class SysID{
    public enum SysIDType{
        Steer,
        Drive,
        X,
        Y,
    }
    SysIdRoutine steerSysId;
    SysIdRoutine driveSysId;
    SysIdRoutine xSpeedSysId;
    SysIdRoutine ySpeedSYSId;
    SwerveModule.ModuleName[] moduleNames = SwerveModule.ModuleName.values();

    public SysID(DriveBase driveBase){
        steerSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("SysIDSteerState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for(int i = 0; i < 4; i++) driveBase.runVoltageSteer(voltage, moduleNames[i]);
                        },
                        null,
                        driveBase,
                        "steerSysId"
                ));

        driveSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(3), null, (state) -> Logger.recordOutput("SysIDDriveState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for(int i = 0; i < 4; i++){
                                driveBase.runVoltageDrive(voltage, moduleNames[i], new Rotation2d(RobotContainer.driveController.getLeftX(), RobotContainer.driveController.getLeftY()));
                            }
                        },
                        null,
                        driveBase,
                        "driveSysId"
                ));

        xSpeedSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Units.Volts.of(0.5).per(Units.Seconds), Units.Volts.of(3), null, (state) -> Logger.recordOutput("SysIDXYState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> driveBase.drive(voltage.in(Units.Volts), 0, 0),
                        (log) -> Logger.recordOutput("/SysID/velocityX", driveBase.getChassisSpeeds().vxMetersPerSecond),
                        driveBase,
                        "xySpeedSysId"
                ));

        ySpeedSYSId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("SysIDYState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> driveBase.drive(0, voltage.in(Units.Volts), 0),
                        (log) -> Logger.recordOutput("/SysID/velocityY", driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond),
                        driveBase,
                        "ySpeedSysId"
                )
        );
    }

    public Command getSysIDCommand(SysIDType type, boolean isDynamic, boolean isForward){
        return isDynamic ? switch (type) {
            case Steer -> steerSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case Drive -> driveSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case X -> xSpeedSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case Y -> ySpeedSYSId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        }
                : switch (type) {
            case Steer -> steerSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case Drive -> driveSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case X -> xSpeedSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
            case Y -> ySpeedSYSId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        };
    }
}