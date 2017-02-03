package org.usfirst.frc.team1318.robot;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Named;
import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.CANTalonControlMode;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.CANTalonWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.CompressorWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.EncoderWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICANTalon;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICompressor;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IEncoder;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IJoystick;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IMotor;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IPowerDistributionPanel;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.JoystickWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.PowerDistributionPanelWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.VictorWrapper;
import org.usfirst.frc.team1318.robot.onemotor.OneMotorController;

import com.google.inject.AbstractModule;
import com.google.inject.Injector;
import com.google.inject.Provides;

public class RobotModule extends AbstractModule
{
    @Override
    protected void configure()
    {
    }

    @Singleton
    @Provides
    public ControllerManager getControllerManager(Injector injector)
    {
        List<IController> controllerList = new ArrayList<>();
        //controllerList.add(injector.getInstance(PowerManager.class));
        //controllerList.add(injector.getInstance(VisionManager.class));
        //controllerList.add(injector.getInstance(CompressorController.class));
        //controllerList.add(injector.getInstance(PositionManager.class));
        //controllerList.add(injector.getInstance(DriveTrainController.class));
        controllerList.add(injector.getInstance(OneMotorController.class));
        return new ControllerManager(controllerList);
    }

    @Singleton
    @Provides
    @Named("USER_DRIVER_JOYSTICK")
    public IJoystick getDriverJoystick()
    {
        return new JoystickWrapper(ElectronicsConstants.JOYSTICK_DRIVER_PORT);
    }

    @Singleton
    @Provides
    @Named("USER_CODRIVER_JOYSTICK")
    public IJoystick getCoDriverJoystick()
    {
        return new JoystickWrapper(ElectronicsConstants.JOYSTICK_CO_DRIVER_PORT);
    }

    @Singleton
    @Provides
    @Named("COMPRESSOR")
    public ICompressor getCompressor()
    {
        return new CompressorWrapper(ElectronicsConstants.PCM_B_MODULE);
    }

    @Singleton
    @Provides
    @Named("POWERMANAGER_PDP")
    public IPowerDistributionPanel getPowerManagerPdp()
    {
        return new PowerDistributionPanelWrapper();
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_LEFTMOTOR")
    public IMotor getDriveTrainLeftMotor()
    {
        return new VictorWrapper(ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_RIGHTMOTOR")
    public IMotor getDriveTrainRightMotor()
    {
        return new VictorWrapper(ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_LEFTENCODER")
    public IEncoder getDriveTrainLeftEncoder()
    {
        EncoderWrapper encoder = new EncoderWrapper(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);

        encoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE);

        return encoder;
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_RIGHTENCODER")
    public IEncoder getDriveTrainRightEncoder()
    {
        EncoderWrapper encoder = new EncoderWrapper(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        encoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE);

        return encoder;
    }

    @Singleton
    @Provides
    @Named("ONEMOTOR_MOTOR")
    public ICANTalon getOneMotorMotor()
    {
        CANTalonWrapper master = new CANTalonWrapper(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);
        master.enableBrakeMode(false);
        master.reverseSensor(true);

        CANTalonWrapper follower = new CANTalonWrapper(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL);
        follower.enableBrakeMode(false);
        follower.reverseOutput(true);
        follower.changeControlMode(CANTalonControlMode.Follower);
        follower.set(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            master.changeControlMode(CANTalonControlMode.Speed);
            master.setPIDF(
                TuningConstants.ONEMOTOR_PID_KP,
                TuningConstants.ONEMOTOR_PID_KI,
                TuningConstants.ONEMOTOR_PID_KD,
                TuningConstants.ONEMOTOR_PID_KF);
        }

        return master;
    }
}
