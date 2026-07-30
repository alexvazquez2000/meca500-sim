package com.alex.meca500.api;

/**
 * Signals a REST-level failure that should be reported to the client with a
 * specific HTTP status code and machine-readable error code (e.g. "busy",
 * "not_ready", "out_of_range"), rather than a generic 500.
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public final class ApiException extends RuntimeException {

	private static final long serialVersionUID = 7241945009598720173L;
	public final int statusCode;
	public final String errorCode;

	public ApiException(int statusCode, String errorCode, String message) {
		super(message);
		this.statusCode = statusCode;
		this.errorCode = errorCode;
	}

}
