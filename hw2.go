package hw2

import (
	"github.com/gonum/graph"
	"math"
)

type setNodes struct {
	node graph.Node
	adjacent int
	tent float64
}

// Struct used to send distance updates through channel
type updateDistance struct {
	toIdx           int     // indexOf going to
	newDistance     float64 // distance of new path
	node            graph.Node // from node
}


// Struct used to send distance updates through channel
type distance struct {
	toIdx           int     // indexOf going to
	newDistance     float64 // distance of new path
	currentDistance float64 // previously known shortest distance
	fromIdx         int     // indexOf coming from
	negativeWeight  bool    // does the edge have a negative weight
}

// Calculate the distance between nodes, and send the update to channel c
func calculateDistance(paths Shortest, g graph.Graph, from graph.Node, to graph.Node, c chan distance) {
	fromIdx := paths.indexOf[from.ID()]
	toIdx := paths.indexOf[to.ID()]
	currentDistance := paths.dist[toIdx]
	weight := g.Edge(from, to).Weight()
	negativeWeight := false
	if weight < 0 {
		negativeWeight = true
	}
	newDistance := paths.dist[fromIdx] + weight
	c <- distance{toIdx: toIdx, newDistance: newDistance, currentDistance: currentDistance,
		fromIdx: fromIdx, negativeWeight: negativeWeight}
}

// Apply the bellman-ford algorithm to Graph and return
// a shortest path tree.
//
// Note that this uses Shortest to make it easier for you,
// but you can use another struct if that makes more sense
// for the concurrency model you chose.
func BellmanFord(s graph.Node, g graph.Graph) Shortest {
	// Ensure node is in graph
	if !g.Has(s) {
		return Shortest{from: s}
	}

	// Initialize path
	paths := newShortestFrom(s, g.Nodes())

	// Relax edges and iteration |V| - 1 times
	for iteration := 0; iteration < len(g.Nodes())-1; iteration++ {
		changed := false
		// Every iteration, loop through every vertex
		for _, from := range g.Nodes() {
			// Create channel for update data to be sent back
			c := make(chan distance)
			fromNodes := g.From(from)

			for _, to := range fromNodes {
				// Loop through every node reachble from current node
				// and send result through channel
				go calculateDistance(paths, g, from, to, c)
			}

			for i := 0; i < len(fromNodes); i++ {
				// Get update from channel
				update := <-c
				if update.negativeWeight {
					panic("Bellman ford: negative weight")
				}
				// Update if more optimal route
				if update.newDistance < update.currentDistance {
					changed = true
					paths.dist[update.toIdx] = update.newDistance
					paths.next[update.toIdx] = update.fromIdx
				}
			}
		}
		// If there were no changes on whole iteration, end looping
		if !changed {
			break
		}
	}
	return paths
}

func unionOf(original []graph.Node, ToCheck graph.Node) []graph.Node {
	if len(original) != 0 {
		for i := range original {
			if (original)[i] == ToCheck {
				return original
			}
		}
	}
	
	original = append(original, ToCheck)
	
	return original
}

func unionOfBucket(original []graph.Node, ToCheck []graph.Node) []graph.Node {

	if len(original) == 0 {
		for _, item := range ToCheck {
			original = append(original, item)
		}
	} else {
		m := make(map[graph.Node]bool)
		
		for _, item := range original {
			m[item] = true
		}
		
		for _, item := range ToCheck {
			if _, ok := m[item]; !ok {
				original = append(original, item)
			}
		}
		
		for i := range (original) {
			if (original)[i] == nil {
				original = append(original[:i], original[i+1:]...)
				break
			}
		}
	}

	return original
}

// true = light, false = heavy
func FindRequests(q []graph.Node, kind bool, g graph.Graph, path *Shortest) []setNodes {

    var weight Weighting
	if wg, ok := g.(graph.Weighter); ok {
		weight = wg.Weight
	} else {
		weight = UniformCost(g)
	}
	
	// requests to relax - initialize to empty, should be vertices that are to be relaxed
	var ReqToRelax []setNodes
	
	// for each node in B[i]
	for m := range q {
		// for all nodes adjacent to node in B[i]
		for _, x := range g.From(q[m]) {
			// get weight of edge between the nodes
			w, ok := weight(q[m], x)
			
			// if invalid weight, panic
            if !ok {
				panic("delta step: unexpected invalid weight")
			}
			// panic for negative weight
			if w < 0 {
				panic("delta step: negative edge weight")
			}
	
			// light
			if kind {
				// if weight is less than or equal to delta, it is a light edge
				if w <= 3 {
					var temp setNodes
					temp.node = x
					temp.tent = (*path).dist[(*path).indexOf[q[m].ID()]] + w
					temp.adjacent = (*path).indexOf[q[m].ID()]
					// add to vertices to relax
					ReqToRelax = append(ReqToRelax, temp)
				}
			}
			// heavy
			if !kind {
				// if weight is greater than delta
				if w > 3 {
					var temp setNodes
					temp.node = x
					temp.tent = (*path).dist[(*path).indexOf[q[m].ID()]] + w
					temp.adjacent = (*path).indexOf[q[m].ID()]
					 // add to vertices to relax
					ReqToRelax = append(ReqToRelax, temp)
				}
			}
		}
	}
	
	return ReqToRelax
}

func relax(Req []setNodes, c chan updateDistance) {
	for i := range Req {

		var m graph.Node
		var n float64
		var a int

		m = Req[i].node
		n = Req[i].tent
		a = Req[i].adjacent
		
		c <- updateDistance{toIdx: a, newDistance: n, node: m}
	}
}

// Apply the delta-stepping algorithm to Graph and return
// a shortest path tree.
//
// Note that this uses Shortest to make it easier for you,
// but you can use another struct if that makes more sense
// for the concurrency model you chose.

// delta is picked to be a fixed value of 3 - needs to be between 1 and n * max edge weight
// B[i] has vertices with labels in [i*delta, (i+1)*delta)
func DeltaStep(s graph.Node, g graph.Graph) Shortest {
    if !g.Has(s) {
		return Shortest{from: s}
	}

	// Initialize path
	path := newShortestFrom(s, g.Nodes())
	
	for p := range g.Nodes() {
		path.dist[p] = math.Inf(1)
	}
	
	var R []graph.Node
	var B [][]graph.Node
	
	path.dist[path.indexOf[s.ID()]] = 0
	
	B = append(B, []graph.Node{s})
	
	i := 0
	
	for len(B) != 0 {
		R = nil
		for len(B[i]) != 0 {
			Req := FindRequests(B[i], true, g, &path)
			R = unionOfBucket(R, B[i])	
			B[i] = nil
			// in parallel
			c := make (chan updateDistance)
			go relax(Req, c)
			if len(Req) != 0 {
				for i := 0; i < len(Req); i++ {	
					update := <- c
					for len(B) <= int(update.newDistance/3) {
						B = append(B, []graph.Node{nil})
						B[len(B)-1] = nil
					}
					if math.IsInf(path.dist[path.indexOf[update.node.ID()]], 1) {
						// put in new bucket
						B[int(update.newDistance/3)] = unionOf(B[int(update.newDistance/3)], update.node)
						path.set(path.indexOf[update.node.ID()], update.newDistance, update.toIdx)
					} else {
						if update.newDistance < path.dist[path.indexOf[update.node.ID()]] {
							// remove from bucket
							if len(B[int(path.dist[path.indexOf[update.node.ID()]]/3)]) != 0 {
								if len(B[int(path.dist[path.indexOf[update.node.ID()]]/3)]) == 1 {
									if B[int(path.dist[path.indexOf[update.node.ID()]]/3)][0] == update.node {
										B[int(path.dist[path.indexOf[update.node.ID()]]/3)] = nil
									}
								} else {
									for i := range B[int(path.dist[path.indexOf[update.node.ID()]]/3)] {
										if B[int(path.dist[path.indexOf[update.node.ID()]]/3)][i] == update.node {
											B[int(path.dist[path.indexOf[update.node.ID()]]/3)] = append(B[int(path.dist[path.indexOf[update.node.ID()]]/3)][:i], B[int(path.dist[path.indexOf[update.node.ID()]]/3)][i+1:]...)
										}
									}
								}
							}
							// put in new bucket
							B[int(update.newDistance/3)] = unionOf(B[int(update.newDistance/3)], update.node)
							path.set(path.indexOf[update.node.ID()], update.newDistance, update.toIdx)
						}
					}
				}
			}
		}
		Req := FindRequests(R, false, g, &path)
		// in parallel
		c := make (chan updateDistance)
		go relax(Req, c)
		if len(Req) != 0 {
			for i := 0; i < len(Req); i++ {
				update := <- c
				for len(B) <= int(update.newDistance/3) {
					B = append(B, []graph.Node{nil})
					B[len(B)-1] = nil
				}
				if math.IsInf(path.dist[path.indexOf[update.node.ID()]], 1) {
					// put in new bucket
					B[int(update.newDistance/3)] = unionOf(B[int(update.newDistance/3)], update.node)
					path.set(path.indexOf[update.node.ID()], update.newDistance, update.toIdx)
				} else {
					if update.newDistance < path.dist[path.indexOf[update.node.ID()]] {
						// remove from bucket
						if len(B[int(path.dist[path.indexOf[update.node.ID()]]/3)]) != 0 {
							if len(B[int(path.dist[path.indexOf[update.node.ID()]]/3)]) == 1 {
								if B[int(path.dist[path.indexOf[update.node.ID()]]/3)][0] == update.node {
									B[int(path.dist[path.indexOf[update.node.ID()]]/3)] = nil
								}
							} else {
								for i := range B[int(path.dist[path.indexOf[update.node.ID()]]/3)] {
									if B[int(path.dist[path.indexOf[update.node.ID()]]/3)][i] == update.node {
										B[int(path.dist[path.indexOf[update.node.ID()]]/3)] = append(B[int(path.dist[path.indexOf[update.node.ID()]]/3)][:i], B[int(path.dist[path.indexOf[update.node.ID()]]/3)][i+1:]...)
									}
								}
							}
						}
						// put in new bucket
						B[int(update.newDistance/3)] = unionOf(B[int(update.newDistance/3)], update.node)
						path.set(path.indexOf[update.node.ID()], update.newDistance, update.toIdx)
					}
				}
			}
		}
		i = i+1
		empty := true
		for i := range B {
			if B[i] != nil {
				empty = false
			}
		}
		if empty == true {
			B = nil
		}
	}
	
	return path
}

// Runs dijkstra from gonum to make sure that the tests are correct.
func Dijkstra(s graph.Node, g graph.Graph) Shortest {
	return DijkstraFrom(s, g)
}
