package hw2

import (
	"math"

	"github.com/gonum/graph"
)

func reverse(p []graph.Node) {
	for i, j := 0, len(p)-1; i < j; i, j = i+1, j-1 {
		p[i], p[j] = p[j], p[i]
	}
}

// Shortest is a shortest-path tree created by the BellmanFordFrom or DijkstraFrom
// single-source shortest path functions.
type Shortest struct {
	// from holds the source node given to
	// DijkstraFrom.
	from graph.Node

	// nodes hold the nodes of the analysed
	// graph.
	nodes []graph.Node
	// indexOf contains a mapping between
	// the id-dense representation of the
	// graph and the potentially id-sparse
	// nodes held in nodes.
	indexOf map[int]int

	// dist and next represent the shortest
	// paths between nodes.
	//
	// Indices into dist and next are
	// mapped through indexOf.
	//
	// dist contains the distances
	// from the from node for each
	// node in the graph.
	dist []float64
	// next contains the shortest-path
	// tree of the graph. The index is a
	// linear mapping of to-dense-id.
	next []int
}

func newShortestFrom(u graph.Node, nodes []graph.Node) Shortest {
	indexOf := make(map[int]int, len(nodes))
	uid := u.ID()
	for i, n := range nodes {
		indexOf[n.ID()] = i
		if n.ID() == uid {
			u = n
		}
	}

	p := Shortest{
		from: u,

		nodes:   nodes,
		indexOf: indexOf,

		dist: make([]float64, len(nodes)),
		next: make([]int, len(nodes)),
	}
	for i := range nodes {
		p.dist[i] = math.Inf(1)
		p.next[i] = -1
	}
	p.dist[indexOf[uid]] = 0

	return p
}

func (p Shortest) set(to int, weight float64, mid int) {
	p.dist[to] = weight
	p.next[to] = mid
}

// From returns the starting node of the paths held by the Shortest.
func (p Shortest) From() graph.Node { return p.from }

// WeightTo returns the weight of the minimum path to v.
func (p Shortest) WeightTo(v graph.Node) float64 {
	to, toOK := p.indexOf[v.ID()]
	if !toOK {
		return math.Inf(1)
	}
	return p.dist[to]
}

// To returns a shortest path to v and the weight of the path.
func (p Shortest) To(v graph.Node) (path []graph.Node, weight float64) {
	to, toOK := p.indexOf[v.ID()]
	if !toOK || math.IsInf(p.dist[to], 1) {
		return nil, math.Inf(1)
	}
	from := p.indexOf[p.from.ID()]
	path = []graph.Node{p.nodes[to]}
	for to != from {
		path = append(path, p.nodes[p.next[to]])
		to = p.next[to]
	}
	reverse(path)
	return path, p.dist[p.indexOf[v.ID()]]
}
