package org.usfirst.frc.team1318.robot;

import org.usfirst.frc.team1318.robot.Compressor.CompressorComponent;
import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.General.PositionManager;
import org.usfirst.frc.team1318.robot.General.PowerManager;
import org.usfirst.frc.team1318.robot.OneMotor.OneMotorComponent;

public class ComponentManager
{
    private CompressorComponent compressorComponent;
    private DriveTrainComponent driveTrainComponent;

    private PowerManager powerManager;
    private PositionManager positionManager;

    private OneMotorComponent oneMotorComponent;

    public ComponentManager()
    {
        this.compressorComponent = new CompressorComponent();
        this.driveTrainComponent = new DriveTrainComponent();
        this.oneMotorComponent = new OneMotorComponent();
        this.powerManager = new PowerManager();
        this.positionManager = new PositionManager(this.driveTrainComponent);
    }

    public CompressorComponent getCompressor()
    {
        return this.compressorComponent;
    }

    public DriveTrainComponent getDriveTrain()
    {
        return this.driveTrainComponent;
    }

    public PowerManager getPowerManager()
    {
        return this.powerManager;
    }

    public PositionManager getPositionManager()
    {
        return this.positionManager;
    }

    public OneMotorComponent getOneMotor()
    {
        return this.oneMotorComponent;

    }
}
