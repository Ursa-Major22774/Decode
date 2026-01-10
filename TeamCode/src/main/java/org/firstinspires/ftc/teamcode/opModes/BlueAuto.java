package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Blue Auto", group = "Competition Autos")
@Configurable
public class BlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Turret turret;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        turret = new Turret(hardwareMap);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Configurable
    public static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(57.180, 87.462)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.180, 87.462),
                                    new Pose(40.026, 71.292)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(225))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch(pathState) {
            case 1:
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() < 5) {turret.aimAndReady(false); turret.update(Utilities.getBatteryVoltage(hardwareMap));}
                else if (pathTimer.getElapsedTimeSeconds() >= 5 && pathTimer.getElapsedTimeSeconds() < 6)
                    {turret.shoot(); turret.update(Utilities.getBatteryVoltage(hardwareMap));}
                else if ((!follower.isBusy()) && pathTimer.getElapsedTimeSeconds() > 10){
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    pathState = 0;
                }
                break;
        }

        return pathState;
    }

    public void setPathState (int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
