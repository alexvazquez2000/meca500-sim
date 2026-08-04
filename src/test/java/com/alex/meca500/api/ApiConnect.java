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
		
		postCall(baseUrl + "/connect", "{}", 10);
		postCall(baseUrl + "/activate-and-home", "{}", 10);
		
//		getCall(baseUrl + "/status", new HashMap<>());

		postCall(baseUrl + "/move-joints", """
				{"j1":0,"j2":0,"j3":0,"j4":0,"j5":0,"j6":0}""", 2000);

		postCall(baseUrl + "/move-joints", """
				{"j1":0,"j2":0,"j3":0,"j4":0,"j5":0,"j6":-130}""", 2000);

		getCall(baseUrl + "/pose", new HashMap<>());

		//rise to safe Z before any lateral movement
		postCall(baseUrl + "/move-lin", """
				{"x": 122.79445, "y": 8.32521, "z": 227.89921, "alpha": -180.30399, "beta": 0.9355, "gamma": 3.04982}""", 1000);

//		//Start to descend
//		postCall(baseUrl + "/move-lin", """
//				{"x": 120.79445, "y": -8.32521, "z": 227.89921, "alpha": -180.30399, "beta": 0.9355, "gamma": 3.04982}""", 1000);
//
//		postCall(baseUrl + "/move-lin", """
//				{"x": 119.83676, "y": 3.15258, "z": 218.63574, "alpha": 180.0, "beta": 0.0, "gamma": 2.0}""", 1000);
//
		
		//119.83677,3.15258,218.63574,-180.0,0.0,2.0
//		postCall(baseUrl + "/move-joints", """
//				{"j1":10,"j2":10,"j3":10,"j4":10,"j5":10,"j6":10}
//				""", 1000);
//		getCall(baseUrl + "/joints", new HashMap<>());
//		getCall(baseUrl + "/status", new HashMap<>());
//		
//		postCall(baseUrl + "/move-joints", """
//				{"j1":20,"j2":20,"j3":20,"j4":20,"j5":20,"j6":20}
//				""");
		
		getCall(baseUrl + "/joints", new HashMap<>());
		
		postCall(baseUrl + "/disconnect", "{}",10);
		
	}

	private static void postCall(String url, String jsonParameters, int sleepMillisecs) {
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

			Thread.sleep(sleepMillisecs);
			
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
