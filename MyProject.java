// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  // As per project spec
  public MyProject () {}

  /**
   * Checks whether all of the devices in the network are connected using BFS.
   * Complexity: O(N).
   * 
   * @param adjlist the adjacency list of the graph
   * 
   * @return whether or not all of the devices are connected in the network
   */
  public boolean allDevicesConnected(int[][] adjlist) {
    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];

    // An arbitrary starting vertex
    queue.add(0);
    visited[0] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      visited[current] = true;

      for (int vertex : adjlist[current]) {
        if (vertex != current && !visited[vertex] && !queue.contains(vertex)) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    for(boolean v : visited) {
      if(!v) return false;
    }

    return true;
  }

  /**
   * Computes (using BFS) all possible paths between two vertices in the graph.
   * Complexity: O(N).
   * 
   * @param adjlist the adjacency list of the graph
   * @param src the source vertex
   * @param dst the target vertex
   * 
   * @return the number of paths
   */
  public int numPaths(int[][] adjlist, int src, int dst) {
    if (src == dst) return 1;

    Queue<Integer> queue = new LinkedList<>();
    int deviceCount = adjlist.length;
    boolean[] visited = new boolean[deviceCount];

    int[] parent = new int[deviceCount];
    Arrays.fill(parent, -1);

    queue.add(src);
    visited[src] = true;
    parent[src] = src;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      visited[current] = true;

      for (int vertex : adjlist[current]) {
        if (vertex != current && !visited[vertex] && !queue.contains(vertex)) {
          parent[vertex] = current;
          queue.add(vertex);
        }
      }
    }

    /*
     * Kinda stuck here
     * 
    System.out.print("\nParents: ");
    int count = 0;
    for (int i = 0; i < deviceCount; i++) {
      System.out.print(parent[i] + ", ");
      if (parent[i] != -1 && parent[src] == parent[dst]) {
        count++;
      }
    }
    System.out.println();
    */

    return count;

    //not good complexity -dfs variant and  remove from visited to find every single path then count.
    //this bfs doesnt work bc it forms a spanning tree -> 1 path
    // if we just follow each path out from the source and keep in array and keep following every path (not necessarilly shortest) then count everytime we hit it
    //dont keep track of visited maybe?
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

    // Run Dijkstra's on the graph to get the distances from the source to all nodes
    int[] distances = SSSP(adjlist, src);
    
    for (int i = 0; i < queries.length; i++) {
      short[] subnet = queries[i];
      // Number of devices in the current subnet
      int numberOfDestinations = 0;
 
      // Contains all the nodes in the subnet with priority = distance
      // from the source.
      PriorityQueue<Node> deviceInfo = new PriorityQueue<>();

      // 0 if the device is in the subnet, 1 o/w.
      BitSet inSubnet = new BitSet(deviceCount);

      for (int j = 0; j < deviceCount; j++) {
        short[] device_address = addrs[j];
        for (int k = 0; k < subnet.length; k++) {
          // If not in the subnet
          if (subnet[k] != device_address[k]) {
            inSubnet.set(j);
          }
        }
      }

      for (int j = 0; j < deviceCount; j++) {
        // If device j is in the subnet
        if (!inSubnet.get(j)) {
          hopsByQuery[i] = distances[j];
          deviceInfo.add(new Node(j, distances[j]));
          numberOfDestinations++;
        }

        if (numberOfDestinations > 1) {
          // Get the minimum distance between the src and the other devices...
          // i.e. the root node in the priority queue.
          hopsByQuery[i] = deviceInfo.peek().priority;
        }
      }
    }

    return hopsByQuery;
  }

  /**
   * Runs Dijkstra's single source shortest path algorithm on the given graph.
   * 
   * @param adjlist the adjacency list of the graph
   * @param src the node to start the search from
   * @return distances from the source to all other nodes
   */
  private int[] SSSP (int[][] adjlist, int src) {
    PriorityQueue<Node> queue = new PriorityQueue<>();
    
    int vertexCount = adjlist.length;
    boolean[] visited = new boolean[vertexCount];
    int[] key = new int[vertexCount];
    Arrays.fill(key, -1);
    
    key[src] = 0;
    queue.add(new Node(src, key[src]));

    while (!queue.isEmpty()) {
      Node current = queue.remove();
      if (!visited[current.vertex]) {
        visited[current.vertex] = true;
        key[current.vertex] = current.priority;

        // For every unvisited neighbouring vertex ``v`` to ``current``
        for (int v : adjlist[current.vertex]) {
          if (!visited[v]) {
            queue.add(new Node(v, 1 + current.priority));
          }
        }
      }
    }

    return key;
  }
  
  public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
   //Bellman Ford or Floyd Warshall depending on the complexity (VE vs V^3)
    /* speeds[i][j] relates to the device at adjList[i][j]
     * instead of smallest path, we need biggest path eg MAX speed
     * can travel multiple paths at once and can be asymetric down a link -> check both ways?
     *if (src == dst){return -1}
     *dijkstra again (or similar) to grab the max total speed from the source to destination.
     */

      // Edmonds–Karp algorithm is best complexity
      //https://en.wikipedia.org/wiki/Edmonds%E2%80%93Karp_algorithm

    // make speeds array and just return distance at dst?
    
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
    boolean reachedDestination = true;

    while (reachedDestination) {
      reachedDestination = false;
      Queue<Integer> queue = new LinkedList<>();
      int[] parent = new int[deviceCount];
      boolean[] visited = new boolean[deviceCount];
      
      queue.offer(src);
      visited[src] = true;

      while (!queue.isEmpty()) {
        int current = queue.poll();
        if (current == dst) {
          reachedDestination = true;
          break;
        }

        for (int i = 0; i < deviceCount; i++) {
          if (!visited[i] && speedsMatrix[current][i] > flow[current][i]) {
            queue.offer(i);
            visited[i] = true;
            parent[i] = current;
          }

        }
      }

      // Terminate - we have not reached the destination
      if (!reachedDestination) break;

      // Reached destination
      int temp = speedsMatrix[parent[dst]][dst] - flow[parent[dst]][dst];
      for (int i = dst; i != src; i = parent[i]) {
        temp = Math.min(temp, (speedsMatrix[parent[i]][i] - flow[parent[i]][i]));
      }
        
      for (int i = dst; i != src; i = parent[i]) {
        flow[parent[i]][i] += temp;
        flow[i][parent[i]] -= temp;
      }
    } 

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
