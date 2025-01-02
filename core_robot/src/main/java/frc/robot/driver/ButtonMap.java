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
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
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
            AnalogAxis.XBONE_RSX,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            -TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.TANK_DRIVETRAIN_EXPONENTIAL),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainSlowMode,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_X_BUTTON,
        //     EnumSet.noneOf(Shift.class),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple),

        // intake operations
        new DigitalOperationDescription(
            DigitalOperation.IntakeExtend,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.IntakeRetract,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),

        // stinger operations
        new DigitalOperationDescription(
            DigitalOperation.StingerOut,
            UserInputDevice.Driver,
            0, // dpad-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.StingerIn,
            UserInputDevice.Driver,
            180, // dpad-down
            EnumSet.of(Shift.DriverDebug),
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
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
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

        // Orientation macros.  Does Gimli have a NavX or Pigeon?
        // new MacroOperationDescription(
        //     MacroOperation.FaceForward,
        //     UserInputDevice.Driver,
        //     0, // DPAD-up
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(0),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurn,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.FaceLeft,
        //     UserInputDevice.Driver,
        //     270, // DPAD-left
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(90),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurn,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.FaceRight,
        //     UserInputDevice.Driver,
        //     90, // DPAD-right
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(-90),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurn,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.FaceBackward,
        //     UserInputDevice.Driver,
        //     180, // DPAD-down
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(180),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurn,
        //     }),

        // Intake macros
        new MacroOperationDescription(
            MacroOperation.IntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new IntakeSpinTask(60.0, false),
            new IOperation[]
            {
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
                DigitalOperation.ShooterLowerKicker,
            }),
        new MacroOperationDescription(
            MacroOperation.IntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new IntakeSpinTask(60.0, true),
            new IOperation[]
            {
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
                DigitalOperation.ShooterLowerKicker,
            }),

        // Shooter macros
        new MacroOperationDescription(
            false,
            MacroOperation.SpinClose,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ShooterKickerTask(TuningConstants.SHOOTER_LOWER_KICKER_DURATION, true),
                new ShooterSpinUpTask(TuningConstants.SHOOTER_SPIN_UP_DURATION, TuningConstants.SHOOTER_CLOSE_SHOT_VELOCITY, false)),
            new IOperation[]
            {
                DigitalOperation.ShooterLowerKicker,
                DigitalOperation.ShooterSpin,
                AnalogOperation.ShooterSpeed,
                DigitalOperation.ShooterExtendHood,
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
            }),
        new MacroOperationDescription(
            false,
            MacroOperation.SpinMiddle,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ShooterKickerTask(TuningConstants.SHOOTER_LOWER_KICKER_DURATION, true),
                new ShooterSpinUpTask(TuningConstants.SHOOTER_SPIN_UP_DURATION, TuningConstants.SHOOTER_MIDDLE_SHOT_VELOCITY)),
            new IOperation[]
            {
                DigitalOperation.ShooterLowerKicker,
                DigitalOperation.ShooterSpin,
                AnalogOperation.ShooterSpeed,
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
            }),
        new MacroOperationDescription(
            false,
            MacroOperation.SpinFar,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ShooterKickerTask(TuningConstants.SHOOTER_LOWER_KICKER_DURATION, true),
                new ShooterSpinUpTask(TuningConstants.SHOOTER_SPIN_UP_DURATION, TuningConstants.SHOOTER_FAR_SHOT_VELOCITY, true)),
            new IOperation[]
            {
                DigitalOperation.ShooterLowerKicker,
                DigitalOperation.ShooterSpin,
                AnalogOperation.ShooterSpeed,
                DigitalOperation.ShooterExtendHood,
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
            }),
        new MacroOperationDescription(
            MacroOperation.Shoot,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            0.5,
            1.0,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ShooterKickerTask(TuningConstants.SHOOTER_FIRE_DURATION, false),
                new ShooterSpinDownTask(TuningConstants.SHOOTER_SPIN_UP_DURATION)),
            new IOperation[]
            {
                DigitalOperation.ShooterLowerKicker,
                DigitalOperation.ShooterSpin,
                AnalogOperation.ShooterSpeed,
                DigitalOperation.ShooterExtendHood,
                DigitalOperation.IntakeRotatingIn,
                DigitalOperation.IntakeRotatingOut,
                DigitalOperation.IntakeExtend,
                DigitalOperation.IntakeRetract,
            }),

        // new MacroOperationDescription(
        //     MacroOperation.FollowPathTest1,
        //     UserInputDevice.Test1,
        //     0,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new FollowPathTask("goLeft32inForward18in")
        //     ),
        //     new IOperation[]
        //     {
        //         DigitalOperation.PositionResetFieldOrientation,
        //         DigitalOperation.PositionResetRobotLevel,
        //         AnalogOperation.PositionStartingAngle,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainTurn,
        //         AnalogOperation.DriveTrainLeftPosition,
        //         AnalogOperation.DriveTrainRightPosition,
        //         AnalogOperation.DriveTrainLeftVelocity,
        //         AnalogOperation.DriveTrainRightVelocity,
        //         AnalogOperation.DriveTrainHeadingCorrection,
        //         AnalogOperation.DriveTrainStartingXPosition,
        //         AnalogOperation.DriveTrainStartingYPosition,
        //         DigitalOperation.DriveTrainSlowMode,
        //         DigitalOperation.DriveTrainEnablePID,
        //         DigitalOperation.DriveTrainDisablePID,
        //         DigitalOperation.DriveTrainSimpleMode,
        //         DigitalOperation.DriveTrainUseBrakeMode,
        //         DigitalOperation.DriveTrainUsePositionalMode,
        //         DigitalOperation.DriveTrainUsePathMode,
        //         DigitalOperation.DriveTrainSwapFrontOrientation,
        //         DigitalOperation.DriveTrainResetXYPosition,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.FollowPathTest2,
        //     UserInputDevice.Test1,
        //     180,
        //     EnumSet.noneOf(Shift.class),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> new FollowPathTask("goBackwards30in"),
        //     new IOperation[]
        //     {
        //         DigitalOperation.PositionResetFieldOrientation,
        //         DigitalOperation.PositionResetRobotLevel,
        //         AnalogOperation.PositionStartingAngle,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainTurn,
        //         AnalogOperation.DriveTrainLeftPosition,
        //         AnalogOperation.DriveTrainRightPosition,
        //         AnalogOperation.DriveTrainLeftVelocity,
        //         AnalogOperation.DriveTrainRightVelocity,
        //         AnalogOperation.DriveTrainHeadingCorrection,
        //         AnalogOperation.DriveTrainStartingXPosition,
        //         AnalogOperation.DriveTrainStartingYPosition,
        //         DigitalOperation.DriveTrainSlowMode,
        //         DigitalOperation.DriveTrainEnablePID,
        //         DigitalOperation.DriveTrainDisablePID,
        //         DigitalOperation.DriveTrainSimpleMode,
        //         DigitalOperation.DriveTrainUseBrakeMode,
        //         DigitalOperation.DriveTrainUsePositionalMode,
        //         DigitalOperation.DriveTrainUsePathMode,
        //         DigitalOperation.DriveTrainSwapFrontOrientation,
        //         DigitalOperation.DriveTrainResetXYPosition,
        //     }),
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