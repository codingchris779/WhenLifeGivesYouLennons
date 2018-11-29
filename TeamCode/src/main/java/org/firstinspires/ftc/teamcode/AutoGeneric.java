package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebHandlers;

/**
 * A class to run Autonomous given a strategy.
 */
public class AutoGeneric{

    public enum StartPos {DEPOT, CRATER, DOUBLESAMPLING};
    private StartPos startZone;
    private OpMode opMode;
    private Telemetry telemetry;
    private Navigation nav;

    /**
     * The constructor method that contains everything to run in initialization.
     * @param startZone - StartPos enumerator. Tells which strategy to run. Options are DEPOT, CRATER, or DOUBLESAMPLING.
     * @param opMode - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry - Telemetry of the current OpMode, used to output data to the screen.
     */
    public AutoGeneric(StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.OpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.startZone = startZone;
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Navigation(opMode, telemetry,true);
        nav.hold(0.1f);
       // nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
    }

    /**
     * Run this to run Autonomous.
     */
    public void runOpMode() {
        //-----crater depot run-----//
        if(startZone == StartPos.CRATER) {
            nav.updateCubePos();
            //-----unhooking-----//
            nav.pointTurnRelative(-90f);
            nav.holdForDrive();
            nav.goDistance(13f);
            nav.holdForDrive();
            switch(nav.getCubePos()) {
                case LEFT:
                    nav.pointTurnRelative(55f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-20f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(38f);
                    nav.holdForDrive();
                    nav.goDistance(50f);
                    nav.holdForDrive();
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-50f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-20f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(135f);
                    nav.holdForDrive();
                    nav.goDistance(47f);
                    nav.holdForDrive();
                    break;
                default:
                    nav.goDistance(15f);
                    nav.holdForDrive();
                    nav.goDistance(-15f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(90f);
                    nav.holdForDrive();
                    nav.goDistance(49f);
                    nav.holdForDrive();
                    break;
            }
            nav.pointTurnRelative(-135f);
            nav.holdForDrive();
            nav.goDistance(-40f);
            nav.holdForDrive();
            nav.setTeamMarker(0.8f);
            nav.hold(1);
            nav.goDistance(63f);
            nav.holdForDrive();
          //  nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
            nav.hold(5);
           // nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
            nav.hold(2);
        }

        //-----crater doublesampling and depot run-----//
        else if(startZone == StartPos.DOUBLESAMPLING) {
            nav.updateCubePos();
            //-----unhooking-----//
            nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
            nav.pointTurnRelative(-90f);
            nav.holdForDrive();
            nav.goDistance(13f);
            nav.holdForDrive();
            switch (nav.getCubePos()) {
                case LEFT:
                    nav.pointTurnRelative(55f);
                    nav.holdForDrive();
                    nav.goDistance(20);
                    nav.holdForDrive();
                    nav.goDistance(-17f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(38f);
                    nav.holdForDrive();
                    nav.goDistance(52f);
                    nav.holdForDrive();
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-45f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-16f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(137f);
                    nav.holdForDrive();
                    nav.goDistance(51f);
                    nav.holdForDrive();
                    break;
                default:
                    nav.goDistance(15f);
                    nav.holdForDrive();
                    nav.goDistance(-13f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(96f);
                    nav.holdForDrive();
                    nav.goDistance(51.5f);
                    nav.holdForDrive();
                    break;
            }
            nav.pointTurnRelative(-135f);
            nav.holdForDrive();
            nav.goDistance(-40f);
            nav.holdForDrive();
            switch (nav.getCubePos()) {
                case LEFT:
                    nav.pointTurnRelative(-80f);
                    nav.holdForDrive();
                    nav.goDistance(30f);
                    nav.holdForDrive();
                    nav.goDistance(-32f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(82f);
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-20f);
                    nav.holdForDrive();
                    nav.goDistance(30f);
                    nav.holdForDrive();
                    nav.goDistance(-26f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(20f);
                    break;
                default: //middle
                    nav.pointTurnRelative(-50f);
                    nav.holdForDrive();
                    nav.goDistance(30f);
                    nav.holdForDrive();
                    nav.goDistance(-30f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(49f);
                    break;
            }
            nav.holdForDrive();
            nav.setTeamMarker(0.8f);
            nav.hold(1);
            nav.goDistance(63f);
            nav.holdForDrive();
           // nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
            nav.hold(5);
            //nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
            nav.hold(2);
        }

        //-----depot depot run-----//
        else if (startZone == StartPos.DEPOT) {
            nav.updateCubePos();
            //-----unhooking-----//
            nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
            nav.pointTurnRelative(-90f);
            nav.holdForDrive();
            nav.goDistance(13f);
            nav.holdForDrive();
            switch(nav.getCubePos()) {
                case LEFT:
                    nav.pointTurnRelative(50f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-20f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(42f);
                    nav.holdForDrive();
                    nav.goDistance(54f);
                    nav.holdForDrive();
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-45f);
                    nav.holdForDrive();
                    nav.goDistance(20f);
                    nav.holdForDrive();
                    nav.goDistance(-18f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(132f);
                    nav.holdForDrive();
                    nav.goDistance(5f);
                    nav.holdForDrive();
                    break;
                default:
                    nav.goDistance(15f);
                    nav.holdForDrive();
                    nav.goDistance(-15f);
                    nav.holdForDrive();
                    nav.pointTurnRelative(90f);
                    nav.holdForDrive();
                    nav.goDistance(50f);
                    nav.holdForDrive();
                    break;
            }

            nav.pointTurnRelative(45f);
            nav.holdForDrive();
            nav.goDistance(-58f);
            nav.holdForDrive();
            nav.pointTurnRelative(90f);
            nav.holdForDrive();
            nav.setTeamMarker(0.8f);
            nav.hold(1);
            nav.pointTurnRelative(-89f);
            nav.holdForDrive();
            nav.goDistance(63f);
            nav.holdForDrive();
            //nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
            nav.hold(5);
            //nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
            nav.hold(2);
        }

        //-----marker deploy and driving to crater-----//
//        nav.setTeamMarker(0.8f);
//        nav.hold(1);
//        nav.goDistance(63f);
//        nav.holdForDrive();
//        //nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
//        nav.hold(5);
//      //  nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT); //breaking crater plane
//        nav.hold(2);
    }
}