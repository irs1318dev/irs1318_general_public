package org.usfirst.frc.team1318.robot.OneMotor;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;
import org.usfirst.frc.team1318.robot.Driver.Driver;
import org.usfirst.frc.team1318.robot.Driver.Operation;

public class OneMotorController implements IController
{
    private PIDHandler PID;
    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private Driver driver;
    private OneMotorComponent component;

    public OneMotorController(OneMotorComponent component)
    {
        this.component = component;
        //        this.createPIDHandler();
    }

    @Override
    public void update()
    {

        // calculate desired power setting for the current mode
        double power;
        //        power = this.calculateVelocityModePowerSetting();
        //
        //        power = this.applyPowerLevelRange(power);

        power = this.driver.getAnalog(Operation.OneMotorPower);

        // apply the power settings to the drivetrain component
        this.component.setPower(power);
    }

    //    private double calculateVelocityModePowerSetting()
    //    {
    //        // velocity goals represent the desired percentage of the max velocity
    //        double velocityGoal = driver.getAnalog(Operation.OneMotorPower);
    //
    //        int currentTicks = this.component.getEncoderTicks();
    //
    //        // decrease the desired velocity based on the configured max power level
    //        velocityGoal = velocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
    //
    //        DashboardLogger.putDouble("Velocity Goal", velocityGoal);
    //
    //        // convert velocity goal to power level...
    //        double power;
    //
    //        power = this.PID.calculateVelocity(
    //            velocityGoal,
    //            currentTicks);
    //
    //        // ensure that our algorithms are correct and don't give values outside
    //        // the appropriate range
    //        power = this.applyPowerLevelRange(power);
    //
    //        return power;
    //    }
    //
    //    private double applyPowerLevelRange(double powerLevel)
    //    {
    //        return Helpers.EnforceRange(powerLevel, POWERLEVEL_MIN, POWERLEVEL_MAX);
    //    }
    //
    //    /**
    //     * create a PIDHandler based on our current settings
    //     */
    //    private void createPIDHandler()
    //    {
    //        this.PID = new PIDHandler(
    //            TuningConstants.ONEMOTOR_VELOCITY_PID_KP,
    //            TuningConstants.ONEMOTOR_VELOCITY_PID_KI,
    //            TuningConstants.ONEMOTOR_VELOCITY_PID_KD,
    //            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT,
    //            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS_DEFAULT,
    //            -TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
    //            TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL);
    //    }

    @Override
    public void stop()
    {
        this.component.setPower(0);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }
}
