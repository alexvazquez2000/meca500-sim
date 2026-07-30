package com.alex.meca500.api;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

class JsonUtilTest {

	@Test
	void parsesFlatNumberAndBooleanFields() {
		Map<String, Object> obj = JsonUtil.parseFlatObject("{\"j1\": 10.5, \"j2\": -3, \"ok\": true, \"bad\": false}");
		assertEquals(10.5, (Double) obj.get("j1"));
		assertEquals(-3.0, (Double) obj.get("j2"));
		assertEquals(Boolean.TRUE, obj.get("ok"));
		assertEquals(Boolean.FALSE, obj.get("bad"));
	}

	@Test
	void parsesEmptyObject() {
		assertTrue(JsonUtil.parseFlatObject("{}").isEmpty());
		assertTrue(JsonUtil.parseFlatObject("  {  }  ").isEmpty());
	}

	@Test
	void parsesStringFields() {
		Map<String, Object> obj = JsonUtil.parseFlatObject("{\"name\": \"hello \\\"world\\\"\"}");
		assertEquals("hello \"world\"", obj.get("name"));
	}

	@Test
	void rejectsNestedObjects() {
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("{\"a\": {\"b\": 1}}"));
	}

	@Test
	void rejectsArrays() {
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("{\"a\": [1,2,3]}"));
	}

	@Test
	void rejectsMalformedJson() {
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("{\"a\": }"));
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("not json"));
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("{\"a\": 1"));
	}

	@Test
	void rejectsTrailingContent() {
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.parseFlatObject("{\"a\": 1} garbage"));
	}

	@Test
	void getDoubleThrowsOnMissingOrWrongType() {
		Map<String, Object> obj = JsonUtil.parseFlatObject("{\"a\": true}");
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.getDouble(obj, "missing"));
		assertThrows(JsonUtil.JsonParseException.class, () -> JsonUtil.getDouble(obj, "a"));
	}

	@Test
	void getDoubleWithDefaultFallsBackWhenMissing() {
		Map<String, Object> obj = JsonUtil.parseFlatObject("{}");
		assertEquals(42.0, JsonUtil.getDouble(obj, "timeoutMs", 42.0));
	}

	@Test
	void writeFlatObjectRoundTrips() {
		LinkedHashMap<String, Object> fields = new LinkedHashMap<>();
		fields.put("j1", 10.5);
		fields.put("ok", true);
		fields.put("name", "hi \"there\"");
		String json = JsonUtil.writeFlatObject(fields);

		Map<String, Object> parsed = JsonUtil.parseFlatObject(json);
		assertEquals(10.5, (Double) parsed.get("j1"));
		assertEquals(Boolean.TRUE, parsed.get("ok"));
		assertEquals("hi \"there\"", parsed.get("name"));
	}

	@Test
	void writeFlatObjectFormatsWholeNumbersWithoutDecimal() {
		LinkedHashMap<String, Object> fields = new LinkedHashMap<>();
		fields.put("j1", 10.0);
		assertEquals("{\"j1\":10}", JsonUtil.writeFlatObject(fields));
	}
}
