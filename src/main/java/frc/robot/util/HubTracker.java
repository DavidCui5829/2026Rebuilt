package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.time.Instant;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class HubTracker {
    /**
     * Returns an {@link Optional} containing the current {@link Shift}.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Shift> getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) return Optional.empty();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.endTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }

    /**
     * Returns an {@link Optional} containing the current {@link Time} remaining in the current shift.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Time> timeRemainingInCurrentShift() {
        return getCurrentShift().map((shift) -> Seconds.of(shift.endTime - getMatchTime()));
    }

    /**
     * Returns an {@link Optional} containing the next {@link Shift}.
     * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
     */
    public static Optional<Shift> getNextShift() {
        double matchTime = getMatchTime();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.startTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance, Shift shift) {
        Optional<Alliance> autoWinner = getAutoWinner();
        switch (shift.activeType) {
            case BOTH:
                return true;
            case AUTO_WINNER:
                return autoWinner.isPresent() && autoWinner.get() == alliance;
            case AUTO_LOSER:
                return autoWinner.isPresent() && autoWinner.get() != alliance;
            default:
                return false;
        }
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance) {
        Optional<Shift> currentShift = getCurrentShift();
        return currentShift.isPresent() && isActive(alliance, currentShift.get());
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the robot's {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Shift shift) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && isActive(alliance.get(), shift);
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the robot's {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive() {
        Optional<Shift> currentShift = getCurrentShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return currentShift.isPresent() && alliance.isPresent() && isActive(alliance.get(), currentShift.get());
    }

    /**
     * Returns whether the hub is active for the next {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext(Alliance alliance) {
        Optional<Shift> nextShift = getNextShift();
        return nextShift.isPresent() && isActive(alliance, nextShift.get());
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}.
     * Will return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext() {
        Optional<Shift> nextShift = getNextShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return nextShift.isPresent() && alliance.isPresent() && isActive(alliance.get(), nextShift.get());
    }

    /**
     * Returns the {@link Alliance} that won auto as specified by the FMS/Driver Station's game specific message data.
     * Will return {@link Optional#empty()} if no game message or alliance is available.
     */
    public static Optional<Alliance> getAutoWinner() {
        String msg = DriverStation.getGameSpecificMessage();
        char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
        switch (msgChar) {
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.empty();
        }
    }

    /**
     * Counts up from 0 to 160 seconds as match progresses.
     * Returns -1 if not match isn't running or if in between auto and teleop
     */
    public static double getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 20 - DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 160 - DriverStation.getMatchTime();
        }
        return -1;
    }

    /**
     * Represents an alliance shift.<br>
     * <h4>Values:</h4>
     * <ul>
     * <li>{@link Shift#AUTO}</li> (0-20 sec)
     * <li>{@link Shift#TRANSITION}</li> (20-30 sec)
     * <li>{@link Shift#SHIFT_1}</li> (30-55 sec)
     * <li>{@link Shift#SHIFT_2}</li> (55-80 sec)
     * <li>{@link Shift#SHIFT_3}</li> (80-105 sec)
     * <li>{@link Shift#SHIFT_4}</li> (105-130 sec)
     * <li>{@link Shift#ENDGAME}</li> (130-160 sec)
     * </ul>
     */
    public enum Shift {
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(20, 30, ActiveType.BOTH),
        SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
        SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
        SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
        ENDGAME(130, 160, ActiveType.BOTH);

        final int startTime;
        final int endTime;
        final ActiveType activeType;

        private Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }

    // --- Elastic publishing helpers ---

    /**
     * Publish a single status document to an Elasticsearch index.
     * elasticBaseUrl should be like "http://elasticsearch-host:9200" (no trailing slash).
     * index is the index name to post to.
     * Errors are reported to DriverStation as warnings.
     */
    public static void publishStatusToElastic(String elasticBaseUrl, String index) {
        try {
            Optional<Shift> current = getCurrentShift();
            Optional<Shift> next = getNextShift();
            Optional<Alliance> alliance = DriverStation.getAlliance();
            Optional<Alliance> autoWinner = getAutoWinner();

            double matchTime = getMatchTime();
            Double timeRemaining = current.map(s -> s.endTime - matchTime).orElse(null);

            String json = buildStatusJson(Instant.now().toString(), matchTime, current.orElse(null), timeRemaining,
                    next.orElse(null), alliance.orElse(null), autoWinner.orElse(null), isActive());

            String url = elasticBaseUrl + "/" + index + "/_doc";
            sendJsonToElastic(json, url);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to publish HubTracker status to Elastic: " + e.getMessage(), false);
        }
    }

    private static String buildStatusJson(String timestamp, double matchTime, Shift currentShift, Double timeRemaining,
            Shift nextShift, Alliance alliance, Alliance autoWinner, boolean isActive) {
        // Build a compact JSON string (no external dependency)
        StringBuilder sb = new StringBuilder();
        sb.append("{");
        sb.append("\"timestamp\":\"").append(timestamp).append("\",");
        sb.append("\"match_time\":").append(matchTime).append(",");
        sb.append("\"current_shift\":").append(currentShift == null ? "null" : "\"" + currentShift.name() + "\"").append(",");
        sb.append("\"time_remaining_in_current_shift\":").append(timeRemaining == null ? "null" : timeRemaining).append(",");
        sb.append("\"next_shift\":").append(nextShift == null ? "null" : "\"" + nextShift.name() + "\"").append(",");
        sb.append("\"alliance\":").append(alliance == null ? "null" : "\"" + alliance.name() + "\"").append(",");
        sb.append("\"auto_winner\":").append(autoWinner == null ? "null" : "\"" + autoWinner.name() + "\"").append(",");
        sb.append("\"is_active\":").append(isActive);
        sb.append("}");
        return sb.toString();
    }

    private static void sendJsonToElastic(String json, String urlString) throws IOException {
        URL url = new URL(urlString);
        HttpURLConnection con = (HttpURLConnection) url.openConnection();
        try {
            con.setRequestMethod("POST");
            con.setDoOutput(true);
            con.setRequestProperty("Content-Type", "application/json");
            byte[] out = json.getBytes(StandardCharsets.UTF_8);
            con.setFixedLengthStreamingMode(out.length);
            con.connect();
            try (OutputStream os = con.getOutputStream()) {
                os.write(out);
            }
            int code = con.getResponseCode();
            if (code < 200 || code >= 300) {
                DriverStation.reportWarning("Elastic returned HTTP " + code + " for HubTracker publish", false);
            }
        } finally {
            con.disconnect();
        }
    }

    private static final ScheduledExecutorService ELASTIC_PUBLISHER = Executors.newSingleThreadScheduledExecutor();

    /**
     * Start periodic publishing of hub tracker status to Elastic.
     * Returns a ScheduledFuture you can cancel to stop publishing.
     * Example: startPeriodicPublishing("http://es:9200", "hubtracker", 5);
     */
    public static ScheduledFuture<?> startPeriodicPublishing(String elasticBaseUrl, String index, long periodSeconds) {
        return ELASTIC_PUBLISHER.scheduleAtFixedRate(() -> publishStatusToElastic(elasticBaseUrl, index), 0,
                Math.max(1, periodSeconds), TimeUnit.SECONDS);
    }

    /**
     * Stop the publisher thread. Cancels all future periodic publishes.
     */
    public static void stopPeriodicPublishing() {
        ELASTIC_PUBLISHER.shutdownNow();
    }
}