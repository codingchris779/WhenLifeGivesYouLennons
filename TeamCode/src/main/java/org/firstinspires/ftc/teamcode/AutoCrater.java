package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Crater", group="Auto")
public class AutoCrater extends LinearOpMode {
    public void runOpMode() {
        AutoGeneric autoGeneric = new AutoGeneric(AutoGeneric.StartPos.CRATER, this, telemetry);

        waitForStart();

        autoGeneric.runOpMode();
    }
}
