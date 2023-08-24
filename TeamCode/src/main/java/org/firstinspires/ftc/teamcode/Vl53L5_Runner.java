package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Vl53L5_Runner extends LinearOpMode {

    private VL53L5 tempSensor;

    public void runOpMode() throws InterruptedException {
        tempSensor = hardwareMap.get(VL53L5.class, "ToF Sensor");
    }

}
