package com.alex.meca500.api;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Minimal JSON support scoped to exactly what this REST API needs: flat
 * objects of numbers, booleans and strings, with no nesting and no arrays.
 * This is intentionally not a general-purpose JSON library -- any input
 * outside that shape fails closed with a {@link JsonParseException}.
 * 
 * @author Alex Vazquez <vazqueza2000@gmail.com>
 */
public final class JsonUtil {

	private JsonUtil() {}

	public static final class JsonParseException extends RuntimeException {
		private static final long serialVersionUID = -8832750838634858535L;

		public JsonParseException(String message) { super(message); }
	}

	public static Map<String, Object> parseFlatObject(String json) {
		Parser p = new Parser(json);
		Map<String, Object> result = p.parseObject();
		p.skipWhitespace();
		if (!p.atEnd()) throw new JsonParseException("Unexpected trailing content at position " + p.pos);
		return result;
	}

	public static double getDouble(Map<String, Object> obj, String key) {
		Object v = obj.get(key);
		if (!(v instanceof Double)) throw new JsonParseException("Missing or non-numeric field: " + key);
		return (Double) v;
	}

	public static double getDouble(Map<String, Object> obj, String key, double defaultValue) {
		Object v = obj.get(key);
		if (v == null) return defaultValue;
		if (!(v instanceof Double)) throw new JsonParseException("Field is not numeric: " + key);
		return (Double) v;
	}

	public static String writeFlatObject(LinkedHashMap<String, Object> fields) {
		StringBuilder sb = new StringBuilder();
		sb.append('{');
		boolean first = true;
		for (Map.Entry<String, Object> e : fields.entrySet()) {
			if (!first) sb.append(',');
			first = false;
			sb.append('"').append(escape(e.getKey())).append("\":");
			appendValue(sb, e.getValue());
		}
		sb.append('}');
		return sb.toString();
	}

	private static void appendValue(StringBuilder sb, Object v) {
		if (v == null) {
			sb.append("null");
		} else if (v instanceof Boolean b) {
			sb.append(b ? "true" : "false");
		} else if (v instanceof Number n) {
			double d = n.doubleValue();
			if (d == Math.rint(d) && !Double.isInfinite(d)) {
				sb.append((long) d);
			} else {
				sb.append(d);
			}
		} else if (v instanceof String str) {
			sb.append('"').append(escape(str)).append('"');
		} else {
			throw new IllegalArgumentException("Unsupported JSON value type: " + v.getClass());
		}
	}

	private static String escape(String s) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < s.length(); i++) {
			char c = s.charAt(i);
			switch (c) {
				case '"'  -> sb.append("\\\"");
				case '\\' -> sb.append("\\\\");
				case '\n' -> sb.append("\\n");
				case '\r' -> sb.append("\\r");
				case '\t' -> sb.append("\\t");
				default -> {
					if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
					else sb.append(c);
				}
			}
		}
		return sb.toString();
	}

	private static final class Parser {
		private final String s;
		private int pos;

		Parser(String s) { this.s = s == null ? "" : s; this.pos = 0; }

		boolean atEnd() { return pos >= s.length(); }

		void skipWhitespace() {
			while (pos < s.length() && Character.isWhitespace(s.charAt(pos))) pos++;
		}

		char peek() {
			if (pos >= s.length()) throw new JsonParseException("Unexpected end of input");
			return s.charAt(pos);
		}

		void expect(char c) {
			if (atEnd() || s.charAt(pos) != c) throw new JsonParseException("Expected '" + c + "' at position " + pos);
			pos++;
		}

		Map<String, Object> parseObject() {
			skipWhitespace();
			expect('{');
			LinkedHashMap<String, Object> result = new LinkedHashMap<>();
			skipWhitespace();
			if (!atEnd() && peek() == '}') { pos++; return result; }
			while (true) {
				skipWhitespace();
				String key = parseString();
				skipWhitespace();
				expect(':');
				skipWhitespace();
				Object value = parseValue();
				result.put(key, value);
				skipWhitespace();
				if (atEnd()) throw new JsonParseException("Unexpected end of input in object");
				char c = peek();
				if (c == ',') { pos++; continue; }
				if (c == '}') { pos++; break; }
				throw new JsonParseException("Expected ',' or '}' at position " + pos);
			}
			return result;
		}

		Object parseValue() {
			skipWhitespace();
			if (atEnd()) throw new JsonParseException("Unexpected end of input");
			char c = peek();
			if (c == '"') return parseString();
			if (c == 't' || c == 'f') return parseBoolean();
			if (c == '-' || Character.isDigit(c)) return parseNumber();
			throw new JsonParseException("Unsupported or malformed JSON value at position " + pos
					+ " (only flat objects of strings/numbers/booleans are supported)");
		}

		String parseString() {
			expect('"');
			StringBuilder sb = new StringBuilder();
			while (true) {
				if (atEnd()) throw new JsonParseException("Unterminated string");
				char c = s.charAt(pos++);
				if (c == '"') break;
				if (c == '\\') {
					if (atEnd()) throw new JsonParseException("Unterminated escape sequence");
					char esc = s.charAt(pos++);
					switch (esc) {
						case '"'  -> sb.append('"');
						case '\\' -> sb.append('\\');
						case '/'  -> sb.append('/');
						case 'n'  -> sb.append('\n');
						case 'r'  -> sb.append('\r');
						case 't'  -> sb.append('\t');
						case 'b'  -> sb.append('\b');
						case 'f'  -> sb.append('\f');
						case 'u'  -> {
							if (pos + 4 > s.length()) throw new JsonParseException("Invalid unicode escape");
							String hex = s.substring(pos, pos + 4);
							pos += 4;
							sb.append((char) Integer.parseInt(hex, 16));
						}
						default -> throw new JsonParseException("Invalid escape sequence '\\" + esc + "'");
					}
				} else {
					sb.append(c);
				}
			}
			return sb.toString();
		}

		Boolean parseBoolean() {
			if (s.startsWith("true", pos)) { pos += 4; return Boolean.TRUE; }
			if (s.startsWith("false", pos)) { pos += 5; return Boolean.FALSE; }
			throw new JsonParseException("Invalid literal at position " + pos);
		}

		Double parseNumber() {
			int start = pos;
			if (!atEnd() && peek() == '-') pos++;
			while (!atEnd() && Character.isDigit(peek())) pos++;
			if (!atEnd() && peek() == '.') {
				pos++;
				while (!atEnd() && Character.isDigit(peek())) pos++;
			}
			if (!atEnd() && (peek() == 'e' || peek() == 'E')) {
				pos++;
				if (!atEnd() && (peek() == '+' || peek() == '-')) pos++;
				while (!atEnd() && Character.isDigit(peek())) pos++;
			}
			String num = s.substring(start, pos);
			if (num.isEmpty() || num.equals("-")) throw new JsonParseException("Invalid number at position " + start);
			try {
				return Double.parseDouble(num);
			} catch (NumberFormatException e) {
				throw new JsonParseException("Invalid number '" + num + "' at position " + start);
			}
		}
	}

}
