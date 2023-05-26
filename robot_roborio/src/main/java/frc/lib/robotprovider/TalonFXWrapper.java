package frc.lib.robotprovider;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class TalonFXWrapper implements ITalonFX
{
    private static final int pidIdx = 0;
    private static final double timeoutSecs = 0.010;

    final TalonFX wrappedObject;

    private TalonFXConfiguration currentConfiguration;
    private TalonFXConfigurator currentConfigurator;

    private ControlModeValue controlMode;
    private ControlRequest currentControlRequest;
    private int selectedSlot;

    public TalonFXWrapper(int deviceNumber)
    {
        this(new TalonFX(deviceNumber));
    }

    public TalonFXWrapper(int deviceNumber, String canbus)
    {
        this(new TalonFX(deviceNumber, canbus));
    }

    private TalonFXWrapper(TalonFX wrappedObject)
    {
        this.wrappedObject = wrappedObject;

        this.controlMode = ControlModeValue.DutyCycleOut;
        this.currentControlRequest = new DutyCycleOut(0.0);
        this.selectedSlot = 0;
    }

    public void set(double value)
    {
        this.internalSet(this.controlMode, this.selectedSlot, value);
    }

    public void set(TalonXControlMode mode, double value)
    {
        this.internalSet(TalonFXWrapper.getControlMode(mode), this.selectedSlot, value);
    }

    public void set(TalonXControlMode mode, int slotId, double value)
    {
        this.internalSet(TalonFXWrapper.getControlMode(mode), slotId, value);
    }

    private void internalSet(ControlModeValue mode, int slotId, double value)
    {
        switch (mode)
        {
            case DutyCycleOut:
                DutyCycleOut dcRequest;
                if (this.currentControlRequest instanceof DutyCycleOut)
                {
                    dcRequest = (DutyCycleOut)this.currentControlRequest;
                    dcRequest.withOutput(value);
                }
                else
                {
                    dcRequest = new DutyCycleOut(value);
                }

                this.wrappedObject.setControl(dcRequest);
                break;

            case VoltageOut:
                VoltageOut voRequest;
                if (this.currentControlRequest instanceof VoltageOut)
                {
                    voRequest = (VoltageOut)this.currentControlRequest;
                    voRequest.withOutput(value);
                }
                else
                {
                    voRequest = new VoltageOut(value);
                }

                this.wrappedObject.setControl(voRequest);
                break;

            case Follower:
                StrictFollower fRequest;
                if (this.currentControlRequest instanceof StrictFollower)
                {
                    fRequest = (StrictFollower)this.currentControlRequest;
                    fRequest.withMasterID((int)value);
                }
                else
                {
                    fRequest = new StrictFollower((int)value);
                }

                this.wrappedObject.setControl(fRequest);
                break;

            case PositionDutyCycle:
                PositionDutyCycle pdcRequest;
                if (this.currentControlRequest instanceof PositionDutyCycle)
                {
                    pdcRequest = (PositionDutyCycle)this.currentControlRequest;
                    pdcRequest.withPosition(value);
                }
                else
                {
                    pdcRequest = new PositionDutyCycle(value);
                }

                pdcRequest.withSlot(slotId);
                this.wrappedObject.setControl(pdcRequest);
                break;

            case PositionVoltage:
                PositionVoltage pvRequest;
                if (this.currentControlRequest instanceof PositionVoltage)
                {
                    pvRequest = (PositionVoltage)this.currentControlRequest;
                    pvRequest.withPosition(value);
                }
                else
                {
                    pvRequest = new PositionVoltage(value);
                }

                pvRequest.withSlot(slotId);
                this.wrappedObject.setControl(pvRequest);
                break;

            case VelocityDutyCycle:
                VelocityDutyCycle vdcRequest;
                if (this.currentControlRequest instanceof VelocityDutyCycle)
                {
                    vdcRequest = (VelocityDutyCycle)this.currentControlRequest;
                    vdcRequest.withVelocity(value);
                }
                else
                {
                    vdcRequest = new VelocityDutyCycle(value);
                }

                vdcRequest.withSlot(slotId);
                this.wrappedObject.setControl(vdcRequest);
                break;

            case VelocityVoltage:
                VelocityVoltage vvRequest;
                if (this.currentControlRequest instanceof VelocityVoltage)
                {
                    vvRequest = (VelocityVoltage)this.currentControlRequest;
                    vvRequest.withVelocity(value);
                }
                else
                {
                    vvRequest = new VelocityVoltage(value);
                }

                vvRequest.withSlot(slotId);
                this.wrappedObject.setControl(vvRequest);
                break;

            case MotionMagicDutyCycle:
                MotionMagicDutyCycle mmdcRequest;
                if (this.currentControlRequest instanceof MotionMagicDutyCycle)
                {
                    mmdcRequest = (MotionMagicDutyCycle)this.currentControlRequest;
                    mmdcRequest.withPosition(value);
                }
                else
                {
                    mmdcRequest = new MotionMagicDutyCycle(value);
                }

                mmdcRequest.withSlot(slotId);
                this.wrappedObject.setControl(mmdcRequest);
                break;

            case MotionMagicVoltage:
                MotionMagicVoltage mmvRequest;
                if (this.currentControlRequest instanceof MotionMagicVoltage)
                {
                    mmvRequest = (MotionMagicVoltage)this.currentControlRequest;
                    mmvRequest.withPosition(value);
                }
                else
                {
                    mmvRequest = new MotionMagicVoltage(value);
                }

                mmvRequest.withSlot(slotId);
                this.wrappedObject.setControl(mmvRequest);
                break;

            case CoastOut:
                CoastOut coRequest;
                if (this.currentControlRequest instanceof CoastOut)
                {
                    coRequest = (CoastOut)this.currentControlRequest;
                }
                else
                {
                    coRequest = new CoastOut();
                }

                this.wrappedObject.setControl(coRequest);
                break;

            case StaticBrake:
                StaticBrake sbRequest;
                if (this.currentControlRequest instanceof StaticBrake)
                {
                    sbRequest = (StaticBrake)this.currentControlRequest;
                }
                else
                {
                    sbRequest = new StaticBrake();
                }

                this.wrappedObject.setControl(sbRequest);
                break;

            default:
            case NeutralOut:
                NeutralOut noRequest;
                if (this.currentControlRequest instanceof NeutralOut)
                {
                    noRequest = (NeutralOut)this.currentControlRequest;
                }
                else
                {
                    noRequest = new NeutralOut();
                }

                this.wrappedObject.setControl(noRequest);
                break;
        }
    }

    public void follow(ITalonSRX talonSRX)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new StrictFollower(((TalonSRXWrapper)talonSRX).wrappedObject.getDeviceID());
        this.wrappedObject.setControl((StrictFollower)this.currentControlRequest);
    }

    public void follow(ITalonSRX talonSRX, boolean invertDirection)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new Follower(((TalonSRXWrapper)talonSRX).wrappedObject.getDeviceID(), invertDirection);
        this.wrappedObject.setControl((Follower)this.currentControlRequest);
    }

    public void follow(ITalonFX talonFX)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new StrictFollower(((TalonFXWrapper)talonFX).wrappedObject.getDeviceID());
        this.wrappedObject.setControl((StrictFollower)this.currentControlRequest);
    }

    public void follow(ITalonFX talonFX, boolean invertDirection)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new Follower(((TalonFXWrapper)talonFX).wrappedObject.getDeviceID(), invertDirection);
        this.wrappedObject.setControl((Follower)this.currentControlRequest);
    }

    public void follow(IVictorSPX victorSPX)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new StrictFollower(((VictorSPXWrapper)victorSPX).wrappedObject.getDeviceID());
        this.wrappedObject.setControl((StrictFollower)this.currentControlRequest);
    }

    public void follow(IVictorSPX victorSPX, boolean invertDirection)
    {
        this.controlMode = ControlModeValue.Follower;
        this.currentControlRequest = new Follower(((VictorSPXWrapper)victorSPX).wrappedObject.getDeviceID(), invertDirection);
        this.wrappedObject.setControl((Follower)this.currentControlRequest);
    }

    public void setControlMode(TalonXControlMode mode)
    {
        ControlModeValue newControlMode = TalonFXWrapper.getControlMode(mode);
        if (this.controlMode != newControlMode)
        {
            this.controlMode = newControlMode;
            switch (this.controlMode)
            {
                case DutyCycleOut:
                    this.currentControlRequest = new DutyCycleOut(0.0);
                    break;
                case VoltageOut:
                    this.currentControlRequest = new VoltageOut(0.0);
                    break;
                case Follower:
                    this.currentControlRequest = new StrictFollower(-1);
                    break;
                case PositionDutyCycle:
                    this.currentControlRequest = (new PositionDutyCycle(0.0)).withSlot(this.selectedSlot);
                    break;
                case PositionVoltage:
                    this.currentControlRequest = (new PositionVoltage(0.0)).withSlot(this.selectedSlot);
                    break;
                case VelocityDutyCycle:
                    this.currentControlRequest = (new VelocityDutyCycle(0.0)).withSlot(this.selectedSlot);
                    break;
                case VelocityVoltage:
                    this.currentControlRequest = (new VelocityVoltage(0.0)).withSlot(this.selectedSlot);
                    break;
                case MotionMagicDutyCycle:
                    this.currentControlRequest = (new MotionMagicDutyCycle(0.0)).withSlot(this.selectedSlot);
                    break;
                case MotionMagicVoltage:
                    this.currentControlRequest = (new MotionMagicVoltage(0.0)).withSlot(this.selectedSlot);
                    break;
                case CoastOut:
                    this.currentControlRequest = new CoastOut();
                    break;
                case StaticBrake:
                    this.currentControlRequest = new StaticBrake();
                    break;

                default:
                case NeutralOut:
                    this.currentControlRequest = new NeutralOut();
                    break;
            }
        }
    }

    public void clearRemoteSensor()
    {
        if (this.currentConfiguration == null || this.currentConfigurator == null)
        {
            this.currentConfiguration = new TalonFXConfiguration();
            this.currentConfigurator = this.wrappedObject.getConfigurator();
        }

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.refresh(this.currentConfiguration, TalonFXWrapper.timeoutSecs),
            "TalonFX.clearRemoteSensor-refresh");

        this.currentConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.currentConfiguration.Feedback.FeedbackRemoteSensorID = 0;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(this.currentConfiguration, TalonFXWrapper.timeoutSecs),
            "TalonFX.clearRemoteSensor-apply");
    }

    public void setRemoteSensor(int sensorId)
    {
        if (this.currentConfiguration == null || this.currentConfigurator == null)
        {
            this.currentConfiguration = new TalonFXConfiguration();
            this.currentConfigurator = this.wrappedObject.getConfigurator();
        }

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.refresh(this.currentConfiguration, TalonFXWrapper.timeoutSecs),
            "TalonFX.setRemoteSensor-refresh");

        this.currentConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        this.currentConfiguration.Feedback.FeedbackRemoteSensorID = sensorId;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(this.currentConfiguration, TalonFXWrapper.timeoutSecs),
            "TalonFX.setRemoteSensor-apply");
    }

    public void setGeneralFramePeriod(double frequencyHz)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMS, TalonFXWrapper.timeoutSecs),
            "TalonFX.setGeneralFramePeriod");
    }

    public void setFeedbackUpdateRate(double frequencyHz)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.getPosition().setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            "TalonFX.setFeedbackUpdateRate-Position");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.getVelocity().setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            "TalonFX.setFeedbackFramePeriod-Velocity");
    }

    public void setPIDFFramePeriod(int periodMS)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, periodMS, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDFFramePeriod");
    }

    public void configureVelocityMeasurements(int periodMS, int windowSize)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.valueOf(periodMS), TalonFXWrapper.timeoutSecs),
            "TalonFX.configureVelocityMeasurementPeriod");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementWindow(windowSize, TalonFXWrapper.timeoutSecs),
            "TalonFX.configureVelocityMeasurementWindow");
    }

    public void configureAllowableClosedloopError(int slotId, int error)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configAllowableClosedloopError(slotId, error, TalonFXWrapper.timeoutSecs),
            "TalonFX.configureAllowableClosedloopError");
    }

    public void setSelectedSlot(int slotId)
    {
        this.selectedSlot = slotId;
        switch (this.controlMode)
        {
            case PositionDutyCycle:
                PositionDutyCycle pdcRequest;
                if (this.currentControlRequest instanceof PositionDutyCycle)
                {
                    pdcRequest = (PositionDutyCycle)this.currentControlRequest;
                }
                else
                {
                    pdcRequest = new PositionDutyCycle(0.0);
                    this.currentControlRequest = pdcRequest;
                }

                pdcRequest.withSlot(slotId);
                break;

            case PositionVoltage:
                PositionVoltage pvRequest;
                if (this.currentControlRequest instanceof PositionVoltage)
                {
                    pvRequest = (PositionVoltage)this.currentControlRequest;
                }
                else
                {
                    pvRequest = new PositionVoltage(0.0);
                    this.currentControlRequest = pvRequest;
                }

                pvRequest.withSlot(slotId);
                break;

            case VelocityDutyCycle:
                VelocityDutyCycle vvdcRequest;
                if (this.currentControlRequest instanceof VelocityDutyCycle)
                {
                    vvdcRequest = (VelocityDutyCycle)this.currentControlRequest;
                }
                else
                {
                    vvdcRequest = new VelocityDutyCycle(0.0);
                    this.currentControlRequest = vvdcRequest;
                }

                vvdcRequest.withSlot(slotId);
                break;

            case VelocityVoltage:
                VelocityVoltage vvRequest;
                if (this.currentControlRequest instanceof VelocityVoltage)
                {
                    vvRequest = (VelocityVoltage)this.currentControlRequest;
                }
                else
                {
                    vvRequest = new VelocityVoltage(0.0);
                    this.currentControlRequest = vvRequest;
                }

                vvRequest.withSlot(slotId);
                break;

            case MotionMagicDutyCycle:
                MotionMagicDutyCycle mmdcRequest;
                if (this.currentControlRequest instanceof MotionMagicDutyCycle)
                {
                    mmdcRequest = (MotionMagicDutyCycle)this.currentControlRequest;
                }
                else
                {
                    mmdcRequest = new MotionMagicDutyCycle(0.0);
                    this.currentControlRequest = mmdcRequest;
                }

                mmdcRequest.withSlot(slotId);
                break;

            case MotionMagicVoltage:
                MotionMagicVoltage mmvRequest;
                if (this.currentControlRequest instanceof MotionMagicVoltage)
                {
                    mmvRequest = (MotionMagicVoltage)this.currentControlRequest;
                }
                else
                {
                    mmvRequest = new MotionMagicVoltage(0.0);
                    this.currentControlRequest = mmvRequest;
                }

                mmvRequest.withSlot(slotId);
                break;

            default:
                // no relevant slot for other scenarios...
                break;
        }
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        if (slotId == 0)
        {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = p;
            slot0Configs.kI = i;
            slot0Configs.kD = d;
            slot0Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot0Configs, TalonFXWrapper.timeoutSecs);
        }
        else if (slotId == 1)
        {
            Slot1Configs slot1Configs = new Slot1Configs();
            slot1Configs.kP = p;
            slot1Configs.kI = i;
            slot1Configs.kD = d;
            slot1Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot1Configs, TalonFXWrapper.timeoutSecs);
        }
        else // if (slotId == 2)
        {
            Slot2Configs slot2Configs = new Slot2Configs();
            slot2Configs.kP = p;
            slot2Configs.kI = i;
            slot2Configs.kD = d;
            slot2Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot2Configs, TalonFXWrapper.timeoutSecs);
        }
    }

    public void setMotionMagicPIDF(double p, double i, double d, double f, double velocity, double acceleration, int slotId)
    {

        TalonFXConfiguration talonFXConfigs = this.wrappedObject.get;

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0Configs;
        slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
        slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
        // PID runs on position
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;
        
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagicConfigs;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

        m_talonFX.getConfigurator().apply(talonFXConfigs, 0.050);


        if (slotId == 0)
        {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = p;
            slot0Configs.kI = i;
            slot0Configs.kD = d;
            slot0Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot0Configs, TalonFXWrapper.timeoutSecs);
        }
        else if (slotId == 1)
        {
            Slot1Configs slot1Configs = new Slot1Configs();
            slot1Configs.kP = p;
            slot1Configs.kI = i;
            slot1Configs.kD = d;
            slot1Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot1Configs, TalonFXWrapper.timeoutSecs);
        }
        else // if (slotId == 2)
        {
            Slot2Configs slot2Configs = new Slot2Configs();
            slot2Configs.kP = p;
            slot2Configs.kI = i;
            slot2Configs.kD = d;
            slot2Configs.kV = f;
            this.wrappedObject.getConfigurator().apply(slot2Configs, TalonFXWrapper.timeoutSecs);
        }
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_kP");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_kI");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_kD");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_kF");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configMotionCruiseVelocity(velocity, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_CruiseVelocity");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configMotionAcceleration(acceleration, TalonFXWrapper.timeoutSecs),
            "TalonFX.setMotionMagicPIDF_Acceleration");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_kP");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_kI");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_kD");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_kF");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_IntegralZone(slotId, izone, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_IntegralZone");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configClosedloopRamp(closeLoopRampRate, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPIDF_CloosedloopRamp");
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configForwardLimitSwitchSource(
                source,
                type,
                TalonFXWrapper.timeoutSecs),
            "TalonFX.setForwardLimitSwitch");
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configReverseLimitSwitchSource(
                source,
                type,
                TalonFXWrapper.timeoutSecs),
            "TalonFX.setReverseLimitSwitch");
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        this.wrappedObject.setSensorPhase(invert);
    }

    public void setInvert(TalonFXInvertType invertType)
    {
        com.ctre.phoenix.motorcontrol.TalonFXInvertType ctreInvertType;
        switch (invertType)
        {
            case CounterClockwise:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.CounterClockwise;
                break;

            case FollowMaster:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.FollowMaster;
                break;

            case OpposeMaster:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.OpposeMaster;
                break;

            default:
            case Clockwise:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.Clockwise;
                break;
        }

        this.wrappedObject.setInverted(ctreInvertType);
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
        NeutralMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = NeutralMode.Brake;
        }
        else
        {
            mode = NeutralMode.Coast;
        }

        this.wrappedObject.setNeutralMode(mode);
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVoltageCompSaturation(maxVoltage, TalonFXWrapper.timeoutSecs),
            "TalonFX.setVoltageCompensationSaturation");
        this.wrappedObject.enableVoltageCompensation(enabled);
    }

    public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(enabled, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configSupplyCurrentLimit(config),
            "TalonFX.setSupplyCurrentLimit");
    }

    public void stop()
    {
        this.wrappedObject.set(ControlMode.Disabled, 0.0);
    }

    public void setPosition(double position)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(position, TalonFXWrapper.pidIdx, TalonFXWrapper.timeoutSecs),
            "TalonFX.setPosition");
    }

    public void reset()
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(0.0, TalonFXWrapper.pidIdx, TalonFXWrapper.timeoutSecs),
            "TalonFX.reset");
    }

    public double getPosition()
    {
        return this.wrappedObject.getSelectedSensorPosition(TalonFXWrapper.pidIdx);
    }

    public double getVelocity()
    {
        return this.wrappedObject.getSelectedSensorVelocity(TalonFXWrapper.pidIdx);
    }

    public double getError()
    {
        return this.wrappedObject.getClosedLoopError(TalonFXWrapper.pidIdx);
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        TalonFXSensorCollection collection = this.wrappedObject.getSensorCollection();

        return new TalonXLimitSwitchStatus(
            collection.isFwdLimitSwitchClosed() == 1,
            collection.isRevLimitSwitchClosed() == 1);
    }

    static ControlModeValue getControlMode(TalonXControlMode mode)
    {
        switch (mode)
        {
            case DutyCycleOut:
                return ControlModeValue.DutyCycleOut;

            case VoltageOut:
                return ControlModeValue.VoltageOut;

            case PositionDutyCycle:
                return ControlModeValue.PositionDutyCycle;

            case PositionVoltage:
                return ControlModeValue.PositionVoltage;

            case VelocityDutyCycle:
                return ControlModeValue.VelocityDutyCycle;

            case VelocityVoltage:
                return ControlModeValue.VelocityVoltage;

            case MotionMagicDutyCycle:
                return ControlModeValue.MotionMagicDutyCycle;

            case MotionMagicVoltage:
                return ControlModeValue.MotionMagicVoltage;

            case CoastOut:
                return ControlModeValue.CoastOut;

            case StaticBrake:
                return ControlModeValue.StaticBrake;

            default:
            case NeutralOut:
                return ControlModeValue.NeutralOut;
        }
    }
}
