/**
 *
 */
package roadgraph;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 *
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections of multiple roads. Edges are the roads.
 *
 */
public class MapGraph {

	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint, MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;

	/**
	 * Create a new empty MapGraph
	 *
	 */
	public MapGraph() {
		pointNodeMap = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return pointNodeMap.values().size();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edges.size();
	}

	// For us in DEBUGGING. Print the Nodes in the graph
	public void printNodes() {
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : pointNodeMap.keySet()) {
			MapNode n = pointNodeMap.get(pt);
			System.out.println(n);
		}
	}

	// For us in DEBUGGING. Print the Edges in the graph
	public void printEdges() {
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges) {
			System.out.println(e);
		}

	}

	/**
	 * Add a node corresponding to an intersection
	 *
	 * @param latitude
	 *            The latitude of the location
	 * @param longitude
	 *            The longitude of the location
	 */
	public void addVertex(double latitude, double longitude) {
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point
	 *
	 * @param location
	 *            The location of the intersection
	 */
	public void addVertex(GeographicPoint location) {
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
		} else {
			System.out.println("Warning: Node at location " + location + " already exists in the graph.");
		}

	}

	/**
	 * Add an edge representing a segment of a road. Precondition: The
	 * corresponding Nodes must have already been added to the graph.
	 * 
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 */
	public void addEdge(double lat1, double lon1, double lat2, double lon2, String roadName, String roadType) {
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:" + pt1 + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:" + pt2 + "is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, String roadType) {

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:" + pt1 + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:" + pt2 + "is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, String roadType, double length) {
		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:" + pt1 + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:" + pt2 + "is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point) {
		return pointNodeMap.containsKey(point);
	}

	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName, String roadType, double length) {
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}

	/** Returns the nodes in terms of their geographic locations */
	public Collection<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}

	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Breadth First Search
	 *
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		if (!arePreconditionsFulfilled(start, goal)) {
			return null;
		}

		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);

		// setup to begin BFS
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();

			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode))
				break;

			for (MapNode neighbor : getNeighbors(next)) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}

		// Reconstruct the parent path
		return reconstructPath(parentMap, startNode, endNode, next.equals(endNode));
	}

	private boolean arePreconditionsFulfilled(GeographicPoint start, GeographicPoint goal) {
		if (start == null || goal == null) {
			throw new NullPointerException("Cannot find route from or to null node");
		}
		if (pointNodeMap.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return false;
		}
		if (pointNodeMap.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return false;
		}
		return true;
	}

	private List<GeographicPoint> reconstructPath(HashMap<MapNode, MapNode> parentMap, MapNode start, MapNode goal,
			boolean pathFound) {
		if (!pathFound) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		return searchOnWeightedGraph(start, goal, nodeSearched, (a, b) -> 0.0);
	}

	private List<GeographicPoint> searchOnWeightedGraph(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, BiFunction<MapNode, MapNode, Double> f) {

		if (!arePreconditionsFulfilled(start, goal)) {
			return null;
		}

		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		DistanceComparator comparator = new DistanceComparator(getNumEdges(), comparator);
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();

		initializeDistances();

		startNode.setActualDistance(0.0);
		startNode.setDistance(0.0);

		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.poll();

			if (!visited.contains(next)) {
				visited.add(next);

				// hook for visualization
				nodeSearched.accept(next.getLocation());

				if (next.equals(endNode)) {
					break;
				}

				HashMap<MapNode, Double> distancesMap = calculateDistanesMap(next);

				for (MapNode neighbor : getNeighbors(next)) {
					if (!visited.contains(neighbor)) {
						double distanceOfNode = next.getActualDistance() + distancesMap.get(neighbor);
						if (distanceOfNode < neighbor.getActualDistance()) {
							neighbor.setActualDistance(distanceOfNode);
							distanceOfNode += f.apply(neighbor, endNode);
							neighbor.setDistance(distanceOfNode);
							parentMap.put(neighbor, next);
							toExplore.offer(neighbor);
						}
					}
				}
			}
		}
		
		System.out.println("Visited: " + visited.size());

		// Reconstruct the parent path
		return reconstructPath(parentMap, startNode, endNode, endNode.equals(next));
	}

	private void initializeDistances() {
		for (MapNode m : pointNodeMap.values()) {
			m.setActualDistance(Double.MAX_VALUE);
			m.setDistance(Double.MAX_VALUE);
		}
	}

	private HashMap<MapNode, Double> calculateDistanesMap(MapNode next) {
		HashMap<MapNode, Double> distancesMap = new HashMap<>();

		for (MapEdge e : next.getEdges()) {
			distancesMap.put(e.getEndNode(), e.getLength());
		}
		return distancesMap;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		return searchOnWeightedGraph(start, goal, nodeSearched, (a, b) -> a.getLocation().distance(b.getLocation()));
	}

	// main method for testing
	public static void main(String[] args) {
		/*
		 * Basic testing System.out.print("Making a new map..."); MapGraph
		 * theMap = new MapGraph(); System.out.print(
		 * "DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		 * System.out.println("DONE.");
		 */

		// more advanced testing
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");

		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());

		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));

		System.out.println(route);

		// Use this code in Week 3 End of Week Quiz MapGraph
		theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);

		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

	}

}

class DistanceComparator implements Comparator<MapNode>{
	@Override
	public int compare(MapNode a, MapNode b) {
		double distanceA = a.getActualDistance() + a.getDistance();
		double distanceB = b.getActualDistance() + b.getDistance();
		
		if(distanceA < distanceB) {
			return -1;
		}
		if(distanceB < distanceA) {
			return 1;
		}
		return 0;
	}
}
