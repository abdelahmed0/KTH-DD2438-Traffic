using System;
using System.Collections.Generic;
using UnityEngine;
using Graphs;

/*
 * This code was built and adjusted from the framework of pre-existing code online
 * Copyright (c) 2020 Christopher Boustros <github.com/christopher-boustros>
 * SPDX-License-Identifier: MIT
 */

// This class implements the A* algorithm and is used by the Agent.cs class

namespace Search {
    public class AStar
    {
        /* 
        * Uses the A* algorithm to find the shortest path from a start vertex to a destination vertex from a list of vertices and a list of edges.
        * The shortest path is returned as a list of vertices.
        * The parameters startVertex and destinationVertex are indices of the vertices list. The method stores vertices using their indices.
        * This implementation of the A* algorithm uses the Euclidean distance as the weight of the edges.
        */
        public List<int> regularA(int startVertex, int destinationVertex, List<Point> vertices, List<Tuple<Point, Point>> edges, Func<List<Point>, int, int, int, float> Heuristics=null, int previousStart=-1)
        {
            List<int> open = new List<int>(); // The set of discovered vertices that may need to be expanded
            List<int> parent = new List<int>(); // For vertex v, parent[v] is the parent of v on the shortest path from the start vertex to the destination vertex
            List<float> gScore = new List<float>(); // The list of G scores. gScore[v] is the distance from the start vertex to vertex v
            List<float> hScore = new List<float>(); // The list of H scores. hScore[v] is the distance from vertex v to the destination vertex
            List<float> fScore = new List<float>(); // The list of F scores. fScore[v] = gScore[v] + hScore[v]
            int currentVertex = startVertex; // The vertex the algorithm is currently visiting
            int previousVertex;

            // Initialize the G and F scores with infinite values and the H scores with Euclidean distances
            // and initialize the parent list
            for (int v = 0; v < vertices.Count; v++)
            {
                gScore.Add(float.MaxValue);
                fScore.Add(float.MaxValue);
                hScore.Add(ComputeEuclideanDistance(v, destinationVertex));
                parent.Add(-1); // parent[v] == -1 means that vertex v has no parent
            }

            if (previousStart != -1) {
                parent[startVertex] = previousStart;
            }
            

            // Add the startVertex to the open list and set its scores
            open.Add(startVertex);
            gScore[startVertex] = 0;
            fScore[startVertex] = hScore[startVertex];

            // While the open list is not empty
            while (open.Count > 0)
            {
                currentVertex = findLowestFScoreVertexInOpenList(); // Find the vertex in the open list with the lowest F score
                Debug.Log(currentVertex);
                previousVertex = parent[currentVertex];

                if (currentVertex == destinationVertex)
                { // The destination vertex has been reached
                    return constructPath(currentVertex, previousStart); // Return a list of vertices as the path
                }

                // The destination vertex has not been reached

                open.Remove(currentVertex); // Remove the current vertex from the open list
                List<int> neighbors = FindNeighbors(currentVertex); // Find all neighbors of the currentVertex

                // For each neighbor of the currentVertex
                foreach (int neighbor in neighbors)
                {
                    float tenative_gScore = gScore[currentVertex] + ComputeEuclideanDistance(currentVertex, neighbor);
                    if (Heuristics != null) {
                        tenative_gScore += Heuristics(vertices, previousVertex, currentVertex, neighbor);
                    }
                    if (tenative_gScore < gScore[neighbor])
                    {
                        // The path to the neighbor is better than the previous path
                        // So, use the path to the neighbor instead of the previous path
                        parent[neighbor] = currentVertex; // The parent of the neighbor is now the currentVertex
                        gScore[neighbor] = tenative_gScore; // Compute the neighbor's G score
                        fScore[neighbor] = gScore[neighbor] + hScore[neighbor]; // Compute the neighbor's H score

                        if (!open.Exists(e => e == neighbor))
                        { // If the neighbor is not in the open list
                            open.Add(neighbor); // Add it to the open list
                        }
                    }
                }
            }

            return null; // If we reached the end of the while loop, then a path was not found

            // A helper method that computes and sets the h score of a vertex
            void SetScores(int v) {
                gScore[v] = ComputeEuclideanDistance(startVertex, v);
                fScore[v] = gScore[v] + hScore[v];
            }

            // A helper method that computes the Euclidean distance between two vertices
            float ComputeEuclideanDistance(int v1, int v2)
            {
                return Vector3.Distance(vertices[v2].ToVector3(), vertices[v1].ToVector3());
            }

            // Finds the vertex with the lowest F score in the open list
            int findLowestFScoreVertexInOpenList()
            {
                float lowestFScore = float.MaxValue;
                int lowestScoreVertex = -1;

                foreach (int v in open)
                {
                    // SetScores(v); // Set the scores of v

                    if (fScore[v] < lowestFScore)
                    {
                        lowestFScore = fScore[v];
                        lowestScoreVertex = v;
                    }
                }

                return lowestScoreVertex;
            }

            // A helper method to find the neighbors of a vertex
            List<int> FindNeighbors(int v)
            {
                List<int> neighbors = new List<int>();

                foreach (Tuple<Point, Point> edge in edges)
                {
                    // Convert Points to Vector3 for comparison
                    Vector3 edgeStart = edge.Item1.ToVector3();
                    Vector3 edgeEnd = edge.Item2.ToVector3();

                    // Find the index of edge.Item1 and edge.Item2 in the vertices list
                    int v1 = vertices.FindIndex(e => e.x == edgeStart.x && e.z == edgeStart.z);
                    int v2 = vertices.FindIndex(e => e.x == edgeEnd.x && e.z == edgeEnd.z);

                    if (v == v1)
                    {
                        neighbors.Add(v2);
                    }
                    else if (v == v2)
                    {
                        neighbors.Add(v1);
                    }
                }

                return neighbors;
            }

            // Construct the path as a list of vertex indices
            List<int> constructPath(int current, int previousStart)
            {
                List<int> path = new List<int>(); // The path as a list of vertex indices
                path.Add(current);

                // While the currentVertex has a parent
                while (parent[current] != -1)
                {
                    current = parent[current];
                    path.Add(current);
                }

                path.Reverse(); // Reverse the order of the elements in the path

                if(previousStart != -1) path.RemoveAt(0);

                return path; // Return the path
            }
        }
    }
}