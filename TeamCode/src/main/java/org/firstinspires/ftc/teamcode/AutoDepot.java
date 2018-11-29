package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto Depot", group="Auto")
public class AutoDepot extends LinearOpMode {
    public void runOpMode() {
        AutoGeneric autoGeneric = new AutoGeneric(AutoGeneric.StartPos.DEPOT, this, telemetry);

        waitForStart();

        autoGeneric.runOpMode();
    }
}
