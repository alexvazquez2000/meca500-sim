package com.alex.meca500.api;
import java.io.IOException;
import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class ApiConnect {
	public static String baseUrl = "http://localhost:8500";
	
	public static void main(String[] args) {
		
		// Define your query parameters
		Map<String, String> parameters = new HashMap<>();
		parameters.put("role", "admin");
		parameters.put("status", "active search"); // Contains a space that needs encoding
		//
		
		postCall(baseUrl + "/connect", "{}");
		postCall(baseUrl + "/activate-and-home", "{}");
		
		getCall(baseUrl + "/status", new HashMap<>());
		getCall(baseUrl + "/pose", new HashMap<>());

		postCall(baseUrl + "/move-lin", """
				{"x":120.5189469808946,"y":74.0243366261341,"z":-42.23348365624428,"alpha":-22.718287925248822,"beta":-28.49299635270986,"gamma":-170.54176424182427}
				""");
//
//		postCall(baseUrl + "/move-joints", """
//				{"j1":10,"j2":10,"j3":10,"j4":10,"j5":10,"j6":10}
//				""");
//		getCall(baseUrl + "/joints", new HashMap<>());
//		getCall(baseUrl + "/status", new HashMap<>());
//		
		postCall(baseUrl + "/move-joints", """
				{"j1":20,"j2":20,"j3":20,"j4":20,"j5":20,"j6":20}
				""");
		
		getCall(baseUrl + "/joints", new HashMap<>());
		
		postCall(baseUrl + "/disconnect", "{}");
		
	}

	private static void postCall(String url, String jsonParameters) {
		// Build the HttpClient and HttpRequest
		HttpClient client = HttpClient.newHttpClient();
		HttpRequest request = HttpRequest.newBuilder()
				.uri(URI.create(url))
				.header("Accept", "application/json") // Optional header
				.POST(HttpRequest.BodyPublishers.ofString(jsonParameters))
				.build();

		try {
			// Send the request and capture the response
			HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());

			// Handle the response
			System.out.println("Status Code: " + response.statusCode());
			System.out.println("Response Body: " + response.body());

		} catch (IOException | InterruptedException e) {
			e.printStackTrace();
		}

	}

	private static void getCall(String url, Map<String, String> parameters) {
		// 1. Convert the map into an URL-encoded query string: ?role=admin&status=active+search
		String queryString = parameters.entrySet().stream()
				.map(entry -> URLEncoder.encode(entry.getKey(), StandardCharsets.UTF_8) + "=" +
						URLEncoder.encode(entry.getValue(), StandardCharsets.UTF_8))
				.collect(Collectors.joining("&", "?", ""));

		// 2. Build the HttpClient and HttpRequest
		HttpClient client = HttpClient.newHttpClient();
		HttpRequest request = HttpRequest.newBuilder()
				.uri(URI.create(url + queryString))
				.header("Accept", "application/json") // Optional header
				.GET()
				.build();

		try {
			// 3. Send the request and capture the response
			HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());

			// 4. Handle the response
			System.out.println("GET Status Code: " + response.statusCode());
			System.out.println("GET Response Body: " + response.body());

		} catch (IOException | InterruptedException e) {
			e.printStackTrace();
		}
	}

}
