package frc.robot.driver;

import java.util.EnumSet;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.driver.buttons.*;
import frc.lib.driver.descriptions.*;
import frc.lib.helpers.Helpers;
import frc.robot.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test2Debug,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        // DriveTrain operations
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            1.0,
            TuningConstants.TANK_DRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurn,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.TANK_DRIVETRAIN_EXPONENTIAL),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            0, // DPAD-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        // Vision test operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTagRear,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTagFront,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAbsolutePosition,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        // new MacroOperationDescription(
        //     MacroOperation.PIDLightBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(false),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            90, // DPAD-right
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new PIDBrakeTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                DigitalOperation.DriveTrainSlowMode,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
                DigitalOperation.DriveTrainResetXYPosition,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceForward,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON, // DPAD-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(0),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurn,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceLeft,
            UserInputDevice.Driver,
            270, // DPAD-left
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(90),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurn,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceRight,
            UserInputDevice.Driver,
            90, // DPAD-right
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(-90),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurn,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceBackward,
            UserInputDevice.Driver,
            180, // DPAD-down
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(180),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurn,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test1,
            0,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("goLeft32inForward18in")
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
                DigitalOperation.VisionForceDisable,
            }),

        // Full auton test
        new MacroOperationDescription(
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,
            180,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goBackwards30in"),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
                DigitalOperation.VisionForceDisable,
            }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}