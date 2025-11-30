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

@Autonomous(name = "Blue Auto", group = "Competition Autos")
@Configurable
public class BlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(27.595, 132.191, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(27.595, 132.191), new Pose(43.622, 83.508))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(270))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.622, 83.508), new Pose(17.714, 83.508))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.714, 83.508), new Pose(46.755, 83.749))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.755, 83.749), new Pose(42.778, 59.890))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.778, 59.890), new Pose(18.196, 60.251))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.196, 60.251), new Pose(46.755, 83.749))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.755, 83.749), new Pose(43.249, 35.386))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
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
                if (!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    setPathState(pathState + 1);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path6);
                    setPathState(pathState + 1);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(paths.Path7);
                    setPathState(pathState + 1);
                }
                break;
            case 8:
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
