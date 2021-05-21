// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  // As per project spec
  public MyProject () {}

  /**
   * Checks whether all of the devices in the network are connected using BFS.
   * 
   * @param adjlist the adjacency list of the graph
   * 
   * @return whether or not all of the devices are connected in the network
   */
  public boolean allDevicesConnected(int[][] adjlist) {
    boolean[]visited = bfs(adjlist);
    //transpose the array to check if all nodes connect to 0.  
    boolean[] visited0 = bfs(transpose(adjlist));

    for (int i = 0; i < adjlist.length; i++)
      if (!visited[i] || !visited0[i])
        return false;

    return true;
  }

 /**
  * Performs a simple BFS in O(N) time using a LinkedQueue
  *
  * @param adjacency list to be searched
  * @return a boolean array  of which nodes have been visited
  */
  private boolean[] bfs(int[][] adjList){
    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjList.length];

    // An arbitrary starting vertex
    queue.add(0);
    visited[0] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      visited[current] = true;

      for (int vertex : adjList[current]) {
        if (vertex != current && !visited[vertex] && !queue.contains(vertex)) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }
    return visited;
  }

  /**
  * Transposes an adjacency list.
  *
  * @param adjlist adjacency list to be transposed.
  *
  * @return the transposed adjacency list.
  */

  private int[][] transpose(int[][] adjlist){
    int vertexCount = adjlist.length;

    ArrayList<ArrayList<Integer>> reversed = new ArrayList<>();
    for (int i = 0; i < vertexCount; i++) {
      reversed.add(new ArrayList<Integer>()); 
    }

    for (int u = 0; u < vertexCount; u++) {
      for (int v = 0; v < adjlist[u].length; v++) {
        reversed.get(adjlist[u][v]).add(u); 
      }
    }

    int[][] result = new int [vertexCount][];
    for (int u = 0; u < reversed.size(); u++) {
      result[u] = new int[reversed.get(u).size()];
      for (int v = 0; v < reversed.get(u).size(); v++) {
        result[u][v] = reversed.get(u).get(v);
      }
    }

    return result;
  }

  /**
   * Computes (using BFS) all possible paths between two vertices in the graph.
   * 
   * @param adjlist the adjacency list of the graph
   * @param src the source vertex
   * @param dst the target vertex
   * 
   * @return the number of paths
   */
  public int numPaths(int[][] adjlist, int src, int dst) {
    if (src == dst) return 1;

    Stack<Integer> stack = new Stack<>();
    int deviceCount = adjlist.length;
    boolean[] visited = new boolean[deviceCount];
    int[] parent = new int[deviceCount];

    stack.push(src);
    visited[src] = true;

    int count = 0;
    while (!stack.empty()) {
      int current = stack.pop();
      visited[current] = true;

      if (parent[current] == dst) {
        visited[dst] = true;
      }
      else {
        visited[dst] = false;
      }

      if (current == dst) {
        count++;
      }

      for (int vertex : adjlist[current]) {
        /*
        if (vertex == dst && visited[parent[vertex]]) {
          System.out.println(current + ", " + vertex + ", " + parent[vertex]);
          count++;
        }
        */
        if (vertex != current && !visited[vertex] && !stack.contains(vertex)) {
          parent[vertex] = current;
          stack.push(vertex);
        }
      }
    }
    
    return count;
  }

  /**
   * Computes the minimum number of hops required to reach a device in each subnet query.
   * 
   * @param adjlist The adjacency list describing the links between devices
   * @param addrs The list of IP addresses for each device
   * @param src The source device
   * @param queries The queries to respond to for each subnet
   * 
   * @return An array with the distance to the closest device for each query, or Integer.MAX_VALUE if none are reachable
   */
public int[] closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries) {
    int deviceCount = adjlist.length;
    int[] hopsByQuery = new int[queries.length];
    Arrays.fill(hopsByQuery, Integer.MAX_VALUE);

    int[] distances = bfsDistances(adjlist,src); //shortest path to each node from the source
    
    for (int i = 0; i < queries.length; i++) {//for each query...
      short[] subnet = queries[i];
 
      int min_distance = Integer.MAX_VALUE;

      for (int j = 0; j < deviceCount; j++) {
        short[] device_address = addrs[j];
        boolean not_in_subnet = false; //point not in subnet
        for (int k = 0; k < subnet.length; k++) {
          // If not in the subnet
          if (subnet[k] != device_address[k]) {
            not_in_subnet = true;
          }
        }

        if(!not_in_subnet && distances[j] < min_distance){//if device in subnet and its distance < current min distance
          min_distance = distances[j];

        }
      }
      hopsByQuery[i] = min_distance;
    }    
    
    return hopsByQuery;
  }

  private int[] bfsDistances(int[][] adjList,int src){
    Queue<Integer> queue = new LinkedList<>();
    int[] distances = new int[adjList.length];
    boolean[] visited =  new boolean[adjList.length];

    // An arbitrary starting vertex
    queue.add(src);
    distances[src] = 0;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      visited[current] = true;

      for (int vertex : adjList[current]) {
        if (vertex != current && !visited[vertex] && !queue.contains(vertex)) {
          queue.add(vertex);
          distances[vertex] = distances[current] + 1;
        }
      }
    }
   return distances;
 }
  
  /**
   * Computes the max flow (speed) that can be passed through the source to the destination device.
   * 
   * @param adjlist the adjacency list of the graph
   * @param speeds the max speed of each link
   * @param src the source device
   * @param dst the destination device
   * 
   * @return the maximum speed from source to destination devices
   */
  public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
    // Note: "flow" and "speed" are interchangable...

    if (src == dst) return -1;

    int deviceCount = adjlist.length;

    int[][] speedsMatrix = new int[deviceCount][deviceCount];
    for(int i = 0; i < adjlist.length; i++) {
      for(int j = 0; j <adjlist[i].length; j++) {
        int index = adjlist[i][j];
        speedsMatrix[i][index] = speeds[i][j];
      }
    }

    int[][] flow = new int[deviceCount][deviceCount];

    boolean canFlow = true;
    do {
      canFlow = false;
      Queue<Integer> queue = new LinkedList<>();
      int[] parent = new int[deviceCount];
      boolean[] visited = new boolean[deviceCount];
      
      queue.add(src);
      visited[src] = true;

      while (!queue.isEmpty()) {
        int current = queue.remove();
        if (current == dst) {
          canFlow = true;
        }

        // For every adjacent device, check if has not been visited and
        // that the current flow rate for this device is less than the
        // max flow rate.
        for (int i = 0; i < deviceCount; i++) {
          if (!visited[i] && speedsMatrix[current][i] > flow[current][i]) {
            queue.add(i);
            visited[i] = true;
            parent[i] = current;
          }
        }
      }
      
      // No path to destination
      if (!canFlow)
        break;
      
      // Find new minimum speed by following the path the BFS took from the source
      // to the destination.
      int minSpeed = speedsMatrix[parent[dst]][dst] - flow[parent[dst]][dst];
      for (int i = dst; i != src; i = parent[i]) {
        minSpeed = Math.min(minSpeed, (speedsMatrix[parent[i]][i] - flow[parent[i]][i]));
      }

      // Update flow values - following the same path (backtracking).
      for (int i = dst; i != src; i = parent[i]) {
        flow[parent[i]][i] += minSpeed;
        flow[i][parent[i]] -= minSpeed;
      }

    } while (canFlow);

    // maxSpeed is the sum of all speeds from source to destination
    int maxSpeed = 0;
    for (int i = 0; i < deviceCount; i++)
      maxSpeed += flow[src][i];

    return maxSpeed;
  }

  /**
   * Inner-class that allows for the priority queue to store
   * a given vertex with a priority.
   */
  private class Node implements Comparable<Node> {
    int vertex, priority;

    Node (int vertex, int priority) {
      this.vertex = vertex;
      this.priority = priority;
    }

    public int compareTo (Node other) {
      return Integer.compare(this.priority, other.priority);
    }
  }
}
