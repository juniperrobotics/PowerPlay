package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Guide to write driver: https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver
 * Example rev distance sensor: https://github.com/REVrobotics/2m-Distance-Sensor/blob/master/Source/src/main/java/com/revrobotics/Rev2mDistanceSensor.java
 */

@I2cDeviceType
@DeviceProperties(name = "VL53L5 ToF Sensor", xmlTag = "VL53L5")
public class VL53L5 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x52);

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.STMicroelectronics;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "STMicroelectronics VL53L5 ToF Sensor";
    }

    public VL53L5(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public enum Register{
        WRITE(0x52),
        READ(0x53);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }


    protected short read(Register reg){

        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

}