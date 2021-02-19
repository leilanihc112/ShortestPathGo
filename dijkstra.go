package hw2

import (
	"container/heap"
	"github.com/gonum/graph"
)

// DijkstraFrom returns a shortest-path tree for a shortest path from u to all nodes in
// the graph g. If the graph does not implement graph.Weighter, UniformCost is used.
// DijkstraFrom will panic if g has a u-reachable negative edge weight.
//
// The time complexity of DijkstrFrom is O(|E|.log|V|).
func DijkstraFrom(u graph.Node, g graph.Graph) Shortest {
	if !g.Has(u) {
		return Shortest{from: u}
	}
	var weight Weighting
	if wg, ok := g.(graph.Weighter); ok {
		weight = wg.Weight
	} else {
		weight = UniformCost(g)
	}

	nodes := g.Nodes()
	path := newShortestFrom(u, nodes)

	// Dijkstra's algorithm here is implemented essentially as
	// described in Function B.2 in figure 6 of UTCS Technical
	// Report TR-07-54.
	//
	// This implementation deviates from the report as follows:
	// - the value of path.dist for the start vertex u is initialized to 0;
	// - outdated elements from the priority queue (i.e. with respect to the dist value)
	//   are skipped.
	//
	// http://www.cs.utexas.edu/ftp/techreports/tr07-54.pdf
	Q := priorityQueue{{node: u, dist: 0}}
	for Q.Len() != 0 {
		mid := heap.Pop(&Q).(distanceNode)
		k := path.indexOf[mid.node.ID()]
		if mid.dist > path.dist[k] {
			continue
		}
		for _, v := range g.From(mid.node) {
			j := path.indexOf[v.ID()]
			w, ok := weight(mid.node, v)
			if !ok {
				panic("dijkstra: unexpected invalid weight")
			}
			if w < 0 {
				panic("dijkstra: negative edge weight")
			}
			joint := path.dist[k] + w
			if joint < path.dist[j] {
				heap.Push(&Q, distanceNode{node: v, dist: joint})
				path.set(j, joint, k)
			}
		}
	}

	return path
}

type distanceNode struct {
	node graph.Node
	dist float64
}

// priorityQueue implements a no-dec priority queue.
type priorityQueue []distanceNode

func (q priorityQueue) Len() int            { return len(q) }
func (q priorityQueue) Less(i, j int) bool  { return q[i].dist < q[j].dist }
func (q priorityQueue) Swap(i, j int)       { q[i], q[j] = q[j], q[i] }
func (q *priorityQueue) Push(n interface{}) { *q = append(*q, n.(distanceNode)) }
func (q *priorityQueue) Pop() interface{} {
	t := *q
	var n interface{}
	n, *q = t[len(t)-1], t[:len(t)-1]
	return n
}
