package com.alex.meca500.api;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Map;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import com.alex.meca500.kinematics.TcpPose;

/**
 * Registers the REST control API's HTTP routes on a pre-bound HttpServer.
 * Does not itself start the server -- the caller controls startup ordering.
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public final class RestApiServer {

	private final HttpServer httpServer;
	private final RobotController controller;

	public RestApiServer(HttpServer httpServer, RobotController controller) {
		this.httpServer = httpServer;
		this.controller = controller;
	}

	public void registerRoutes() {
		httpServer.createContext("/connect", wrap("POST", 200, body -> {
			controller.connect();
			return new LinkedHashMap<>();
		}));
		httpServer.createContext("/disconnect", wrap("POST", 200, body -> {
			controller.disconnect();
			return new LinkedHashMap<>();
		}));
		httpServer.createContext("/activate-and-home", wrap("POST", 200, body -> {
			controller.activateAndHome();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("activated", true);
			r.put("homed", true);
			return r;
		}));
		httpServer.createContext("/deactivate", wrap("POST", 200, body -> {
			controller.deactivateRobot();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("activated", false);
			r.put("homed", false);
			return r;
		}));
		httpServer.createContext("/wait-deactivated", wrap("POST", 200, body -> {
			double timeoutMs = JsonUtil.getDouble(body, "timeoutMs", 5000);
			boolean deactivated = controller.waitDeactivated((long) timeoutMs);
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("deactivated", deactivated);
			return r;
		}));
		httpServer.createContext("/move-joints", wrap("POST", 202, body -> {
			double[] targetDeg = new double[6];
			for (int i = 0; i < 6; i++) {
				targetDeg[i] = JsonUtil.getDouble(body, "j" + (i + 1));
			}
			return moveResponse(controller.moveJoints(targetDeg));
		}));
		httpServer.createContext("/move-lin", wrap("POST", 202, body -> {
			TcpPose target = new TcpPose(
					JsonUtil.getDouble(body, "x"), JsonUtil.getDouble(body, "y"), JsonUtil.getDouble(body, "z"),
					JsonUtil.getDouble(body, "alpha"), JsonUtil.getDouble(body, "beta"), JsonUtil.getDouble(body, "gamma"));
			return moveResponse(controller.moveLin(target));
		}));
		httpServer.createContext("/clear-motion", wrap("POST", 200, body -> {
			controller.clearMotion();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("paused", controller.getStatus().paused());
			return r;
		}));
		httpServer.createContext("/resume-motion", wrap("POST", 200, body -> {
			controller.resumeMotion();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("paused", controller.getStatus().paused());
			return r;
		}));
		httpServer.createContext("/status", wrap("GET", 200, body -> {
			RobotController.StatusSnapshot s = controller.getStatus();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("connected", s.connected());
			r.put("activated", s.activated());
			r.put("homed", s.homed());
			r.put("simulation", s.simulation());
			r.put("paused", s.paused());
			r.put("moving", s.moving());
			r.put("eom", s.eom());
			r.put("error", s.error());
			return r;
		}));
		httpServer.createContext("/pose", wrap("GET", 200, body -> {
			TcpPose p = controller.getPose();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			r.put("x", p.x()); r.put("y", p.y()); r.put("z", p.z());
			r.put("alpha", p.alpha()); r.put("beta", p.beta()); r.put("gamma", p.gamma());
			return r;
		}));
		httpServer.createContext("/joints", wrap("GET", 200, body -> {
			double[] j = controller.getJointsDeg();
			LinkedHashMap<String, Object> r = new LinkedHashMap<>();
			for (int i = 0; i < j.length; i++) r.put("j" + (i + 1), j[i]);
			return r;
		}));
	}

	private static LinkedHashMap<String, Object> moveResponse(RobotController.MoveOutcome outcome) {
		switch (outcome) {
			case ACCEPTED -> {
				LinkedHashMap<String, Object> ok = new LinkedHashMap<>();
				ok.put("accepted", true);
				return ok;
			}
			case NOT_READY -> throw new ApiException(409, "not_ready", "Robot is not activated and homed");
			case BUSY -> throw new ApiException(409, "busy", "A motion is already active or paused");
			case OUT_OF_RANGE -> throw new ApiException(400, "out_of_range", "Target joint angle is outside the joint's range");
		}
		throw new ApiException(500, "internal_error", "Unknown move outcome");
	}

	@FunctionalInterface
	private interface Op {
		LinkedHashMap<String, Object> handle(Map<String, Object> body);
	}

	private HttpHandler wrap(String requiredHttpMethod, int successStatus, Op op) {
		return exchange -> {
			try {
				handleExchange(exchange, requiredHttpMethod, successStatus, op);
			} finally {
				exchange.close();
			}
		};
	}

	private void handleExchange(HttpExchange exchange, String requiredHttpMethod, int successStatus, Op op) throws IOException {
		if (!exchange.getRequestMethod().equalsIgnoreCase(requiredHttpMethod)) {
			writeJson(exchange, 405, errorBody("method_not_allowed", "Use " + requiredHttpMethod + " for this endpoint"));
			return;
		}
		try {
			Map<String, Object> body = readBody(exchange);
			LinkedHashMap<String, Object> result = op.handle(body);
			writeJson(exchange, successStatus, result);
		} catch (JsonUtil.JsonParseException e) {
			writeJson(exchange, 400, errorBody("bad_json", e.getMessage()));
		} catch (ApiException e) {
			writeJson(exchange, e.statusCode, errorBody(e.errorCode, e.getMessage()));
		} catch (Exception e) {
			writeJson(exchange, 500, errorBody("internal_error", String.valueOf(e.getMessage())));
		}
	}

	private static Map<String, Object> readBody(HttpExchange exchange) throws IOException {
		InputStream in = exchange.getRequestBody();
		ByteArrayOutputStream buf = new ByteArrayOutputStream();
		in.transferTo(buf);
		String text = buf.toString(StandardCharsets.UTF_8).trim();
		if (text.isEmpty()) return new LinkedHashMap<>();
		return JsonUtil.parseFlatObject(text);
	}

	private static LinkedHashMap<String, Object> errorBody(String errorCode, String message) {
		LinkedHashMap<String, Object> r = new LinkedHashMap<>();
		r.put("error", errorCode);
		r.put("message", message == null ? "" : message);
		return r;
	}

	private static void writeJson(HttpExchange exchange, int statusCode, LinkedHashMap<String, Object> body) throws IOException {
		String json = JsonUtil.writeFlatObject(body);
		byte[] bytes = json.getBytes(StandardCharsets.UTF_8);
		exchange.getResponseHeaders().set("Content-Type", "application/json");
		exchange.sendResponseHeaders(statusCode, bytes.length);
		try (OutputStream os = exchange.getResponseBody()) {
			os.write(bytes);
		}
	}

}
