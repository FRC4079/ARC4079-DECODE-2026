package frc.robot.FlywheelSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class LookupTable extends StateMachine<LookupTable.State> {
    public enum State { DISABLED, ENABLED }

    public static class ShotPoint {
        public final double distanceMeters;
        public final double rpm;
        public final double hoodangle;

        public ShotPoint(double distanceMeters, double rpm, double hoodangle) {
            this.distanceMeters = distanceMeters;
            this.rpm = rpm;
            this.hoodangle = hoodangle;
        }
    }

    private static class TimeOfFlightPoint {
        final double distance;
        final double time;

        TimeOfFlightPoint(double distance, double time) {
            this.distance = distance;
            this.time = time;
        }
    }

    private final DistanceCalc distanceCalc;
    private final Flywheel flywheel;
    private final Hood hood;

    private final List<ShotPoint> points = new ArrayList<>();
    private final List<TimeOfFlightPoint> tofPoints = new ArrayList<>();

    private final Debouncer atGoalDebounce =
        new Debouncer(0.2, Debouncer.DebounceType.kFalling);

    private static final double RPM_TOLERANCE = 75.0;
    private static final double HOOD_TOLERANCE_DEG = 1.0;

    private boolean atGoal = false;

    public LookupTable(DistanceCalc distanceCalc, Flywheel flywheel, Hood hood) {
        super(SubsystemPriority.LOCALIZATION, State.DISABLED);
        this.distanceCalc = distanceCalc;
        this.flywheel = flywheel;
        this.hood = hood;

        addPoint(new ShotPoint(1.0, 2000, 10));
        addPoint(new ShotPoint(2.0, 2500, 12));
        addPoint(new ShotPoint(3.0, 3000, 14));
        addPoint(new ShotPoint(4.0, 3500, 16));
        addPoint(new ShotPoint(5.0, 4000, 18));
        addPoint(new ShotPoint(6.0, 4500, 20));

        addTimeOfFlightPoint(1.0, 0.90);
        addTimeOfFlightPoint(2.0, 1.00);
        addTimeOfFlightPoint(3.0, 1.10);
        addTimeOfFlightPoint(4.0, 1.15);
        addTimeOfFlightPoint(5.0, 1.20);
        addTimeOfFlightPoint(6.0, 1.25);
    }

    public void addPoint(ShotPoint point) {
        points.add(point);
        points.sort(Comparator.comparingDouble(p -> p.distanceMeters));
    }

    private void addTimeOfFlightPoint(double distance, double time) {
        tofPoints.add(new TimeOfFlightPoint(distance, time));
        tofPoints.sort(Comparator.comparingDouble(p -> p.distance));
    }

    private double lookupTimeOfFlight(double distanceMeters) {
        if (tofPoints.isEmpty()) return 0.0;

        if (distanceMeters <= tofPoints.get(0).distance) {
            return tofPoints.get(0).time;
        }

        for (int i = 1; i < tofPoints.size(); i++) {
            var low = tofPoints.get(i - 1);
            var high = tofPoints.get(i);

            if (distanceMeters <= high.distance) {
                double t = (distanceMeters - low.distance) / (high.distance - low.distance);
                return low.time + t * (high.time - low.time);
            }
        }

        return tofPoints.get(tofPoints.size() - 1).time;
    }

    public double[] lookup(double distanceMeters) {
        if (points.isEmpty()) return new double[] {0.0, 0.0};

        if (distanceMeters <= points.get(0).distanceMeters) {
            var p = points.get(0);
            return new double[] {p.rpm, p.hoodangle};
        }

        for (int i = 1; i < points.size(); i++) {
            var low = points.get(i - 1);
            var high = points.get(i);

            if (distanceMeters <= high.distanceMeters) {
                double t = (distanceMeters - low.distanceMeters)
                    / (high.distanceMeters - low.distanceMeters);
                double rpm = low.rpm + t * (high.rpm - low.rpm);
                double hood = low.hoodangle + t * (high.hoodangle - low.hoodangle);
                return new double[] {rpm, hood};
            }
        }

        var last = points.get(points.size() - 1);
        return new double[] {last.rpm, last.hoodangle};
    }


    private double getVelocityCompensatedDistance(double rawDistance) {
        double tof = lookupTimeOfFlight(rawDistance);

        double forwardVel = distanceCalc.getRobotVelocityTowardTargetMetersPerSec();
        double lateralVel = distanceCalc.getRobotLateralVelocityMetersPerSec();
        double angularVel = distanceCalc.getRobotAngularVelocityRadPerSec();

        double compensatedForward = rawDistance + forwardVel * tof;
        double lateralShift = lateralVel * tof;
        double effectiveDistance = Math.hypot(compensatedForward, lateralShift);

        SmartDashboard.putNumber("Shooter/CompForwardDist", compensatedForward);
        SmartDashboard.putNumber("Shooter/LateralShift", lateralShift);
        SmartDashboard.putNumber("Shooter/RotationShiftRad", angularVel * tof);

        return effectiveDistance;
    }


    private double getVelocityCompensatedHood(double rawHoodAngleDeg) {
        double tof = lookupTimeOfFlight(distanceCalc.getDistanceToAllianceTargetMeters());
        double angularVel = distanceCalc.getRobotAngularVelocityRadPerSec();
        return rawHoodAngleDeg + Math.toDegrees(angularVel * tof);
    }

    public void enable() { setStateFromRequest(State.ENABLED); }
    public void disable() { setStateFromRequest(State.DISABLED); }

    @Override
    protected State getNextState(State current) { return current; }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        double rawDistance = distanceCalc.getDistanceToAllianceTargetMeters();
        double compensatedDistance = getVelocityCompensatedDistance(rawDistance);

        double[] result = lookup(compensatedDistance);

        result[1] = getVelocityCompensatedHood(result[1]);

        SmartDashboard.putNumber("Shooter/RawDistance", rawDistance);
        SmartDashboard.putNumber("Shooter/CompDistance", compensatedDistance);
        SmartDashboard.putNumber("Shooter/TargetRPM", result[0]);
        SmartDashboard.putNumber("Shooter/TargetHoodDeg", result[1]);

        if (getState() == State.ENABLED) {
            flywheel.spinFlywheel(result[0]);
            hood.setAngleDegrees(result[1]);

            boolean inTol =
                Math.abs(flywheel.getRpm() - result[0]) <= RPM_TOLERANCE
                && Math.abs(hood.getAngleDegrees() - result[1]) <= HOOD_TOLERANCE_DEG;

            atGoal = atGoalDebounce.calculate(inTol);
            SmartDashboard.putBoolean("Shooter/AtGoal", atGoal);
        }
    }

    public boolean isAtGoal() {
        return atGoal;
    }
}
