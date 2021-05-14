// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  /**
   * Checks whether all of the devices in the network are connected using BFS.
   * 
   * @param adjlist the adjacency list of the graph being checked.
   */
  public boolean allDevicesConnected(int[][] adjlist) {
    // Linear time implementation
    // https://courses.cs.vt.edu/~cs4104/murali/Fall09/lectures/lecture-06-linear-time-graph-algorithms.pdf

    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];

    // An arbitrary starting vertex
    queue.add(0);
    visited[0] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      for (int vertex : adjlist[current]) {
        if (!visited[vertex]) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    for (boolean v : visited)
      if (!v) return false;

    return true;
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
    // This might be helpful for when we want to make it faster:
    // https://www.cs.princeton.edu/~rs/talks/PathsInGraphs07.pdf
    if (src == dst) return 1;

    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];
    int count = 0;

    queue.add(src);
    visited[src] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      for (int vertex : adjlist[current]) {
        if (vertex == dst) {
          count++;
        }
        else if (!visited[vertex]) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    return count;
  }

  public int[] closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries) {
    //Dijkstra's algorithm with binary heap has the complexity we need (wikipedia lmao)
    int deviceCount = adjlist.length;
    int[] hopsByQuery = new int[queries.length];
    Arrays.fill(hopsByQuery, Integer.MAX_VALUE);

    // Run Dijkstra's on the graph
    int[] distances = SSSP(adjlist, src);
    
    ///// TODO: REMOVE /////
    System.out.print("Distances: \t");
    for (int i = 0; i < distances.length; i++)
      System.out.print(distances[i] + ", ");
    System.out.println();
    ////////////////////////
    
    // A dictionary with each element containing (<device_index>, <distance>)
    Hashtable<Integer, Integer> deviceInfo = new Hashtable<>();
    for (int j = 0; j < deviceCount; j++)
      deviceInfo.put(j, distances[j]);
    
    ///// TODO: REMOVE /////
    Set<Integer> set = deviceInfo.keySet();
    for (int key : set) {
      System.out.println("device: " + key + ",\tdistance: " + deviceInfo.get(key));
    }
    ////////////////////////

    for (int i = 0; i < queries.length; i++) {
      short[] subnet = queries[i];
      // Number of devices in the current subnet
      int numberOfDestinations = 0;

      // 0 if in the device is in the subnet, 1 o/w.
      BitSet destinations = new BitSet(deviceCount);

      for (int j = 0; j < deviceCount; j++) {
        short[] device_address = addrs[j];

        for (int k = 0; k < subnet.length; k++) {
          // If not in the subnet
          if (subnet[k] != device_address[k]) {
            // NOTE: we could remove the device from the hashtable but then
            //       we'd have to re-compute the distances for the next query
            //       which would be much slower...
            destinations.set(j);
          }
        }
      }

      for (int j = 0; j < deviceCount; j++) {
        if (!destinations.get(j)) {
          hopsByQuery[i] = distances[j];
          numberOfDestinations++;
        }

        if (numberOfDestinations > 1) {
          // Get the minimum distance between the src and the other devices...
          // TODO: fix...
          Collection<Integer> T = deviceInfo.values();
          hopsByQuery[i] = Collections.min(T);;
        }
      }
    }

    return hopsByQuery;
  }

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
    int[][] flow = new int[deviceCount][deviceCount];
    boolean reachedDestination = false;

    while (!reachedDestination) {
      Queue<Integer> queue = new LinkedList<>();
      int[] parent = new int[deviceCount];
      boolean[] visited = new boolean[deviceCount];
      
      queue.add(src);
      visited[src] = true;

      while (!queue.isEmpty()) {
        int current = queue.remove();
        if (current == dst) {
          reachedDestination = true;
          break;
        }

        for (int i = 0; i < deviceCount; i++) {
          // WARNING: THIS IS SO DUMB I CANT EVEN
          // TODO: get rid of try-catch blocks
          try {
            if (!visited[i] && speeds[current][i] > flow[current][i]) {
              queue.add(i);
              visited[i] = true;
              parent[i] = current;
            }
          } catch (Exception e) {}
        }
      }

      // Terminate - we have not reached the destination
      if (!reachedDestination) break;

      // Reached destination
      try {
        int temp = speeds[parent[dst]][dst] - flow[parent[dst]][dst];
        for (int i = dst; i != src; i = parent[i]) {
          temp = Math.min(temp, (speeds[parent[i]][i] - flow[parent[i]][i]));
        }
        
        for (int i = dst; i != src; i = parent[i]) {
          flow[parent[i]][i] += temp;
          flow[i][parent[i]] -= temp;
        }
      } catch (Exception e) {}
    }

    int maxSpeed = 0;
    for (int i = 0; i < deviceCount; i++) {
      maxSpeed += flow[src][i];
    }
    return maxSpeed;
    
    /*
    int prevSrc = src;
    while(prevSrc != dst){
      for(int i = 0; i < speeds[prevSrc].length; i++){
        int speed1 = speeds[prevSrc][i];
        //which speed in each list is fastest/
        if(speed1 > maxSpeed){
          
        }
        prevSrc = adjlist[prevSrc][i];
        }
    }
    if(src == dst){return -1;}
    return maxSpeed;

    //psuedo codeish
    q = queue();
    int[] parent = int[adjList.length];
    while(!q.isEmpty()){
      current = q.poll()
      for(each edge)
        if(parent[edges] = null and edge != src){
          parent[edge] = edge;
          push edge
        }
          second half lszkhg;kergvb;zbhv
    }
    */
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
