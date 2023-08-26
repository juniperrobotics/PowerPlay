package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Vl53L5_Runner extends LinearOpMode {

    private VL53L5 sensor;

    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(VL53L5.class, "ToF Sensor");
    }

}
