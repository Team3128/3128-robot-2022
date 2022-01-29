package frc.team3128.common.utility;

import edu.wpi.first.wpilibj.DriverStation;

public class Log {
	/**
	 * Log a FATAL error, after which the robot cannot (properly) function. <br>
	 * 
	 * @param category
	 * @param message
	 */
	public static void fatal(String category, String message) {
		log("Fatal", category, message);

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + message, true);
	}

	/**
	 * Log a FATAL error due to an exception, after which the robot cannot
	 * (properly) function. <br>
	 * Prints your message, and the exception's name, message, and stacktrace.
	 * 
	 */
	public static void fatalException(String category, String userMessage, Exception exception) {
		String exceptionMessage = String.format("%s -- %s: %s", userMessage, exception.getClass().getSimpleName(),
				exception.getMessage());
		log("Fatal", category, exceptionMessage);

		exception.printStackTrace();

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + exceptionMessage, true);
	}

	/**
	 * Log a failure which may kill one function or one thread, however the robot as
	 * a whole can keep functioning.
	 * 
	 * @param category
	 * @param message
	 */
	public static void recoverable(String category, String message) {
		log("Recoverable", category, message);

		DriverStation.reportError("Error: " + (message == null ? "null" : message), true);

	}

	/**
	 * Log something which should not happen under normal circumstances and probably
	 * is a bug, but does not cause anything to crash.
	 * 
	 * @param category
	 * @param message
	 */
	public static void unusual(String category, String message) {
		log("Unusual", category, message);
	}

	/**
	 * Log a semi-important message which the user should probably see, but does not
	 * indicate anything is broken.
	 */
	public static void info(String category, String message) {
		log("Info", category, message);
	}

	/**
	 * Log a message which is not important during normal operation, but is useful
	 * if you're trying to debug the robot.
	 * 
	 * @param category
	 * @param message
	 */
	public static void debug(String category, String message) {
		log("Debug", category, message);
	}

	private static void log(String severity, String category, String message) {
		System.out.println(String.format("[%s] [%s] %s", severity, category, message));
	}
}
