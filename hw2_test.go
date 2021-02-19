package hw2

import (
	"math"
	"reflect"
	"testing"

	"github.com/gonum/graph"
)

func Test(t *testing.T) {
	for _, test := range ShortestPathTests {
		g := test.Graph()
		for _, e := range test.Edges {
			g.SetEdge(e)
		}

		var (
			pt Shortest

			panicked bool
		)
		flist := []func(graph.Node, graph.Graph) Shortest{Dijkstra, BellmanFord, DeltaStep}

		for _, f := range flist {
			func() {
				defer func() {
					panicked = recover() != nil
				}()
				pt = f(test.Query.From(), g.(graph.Graph))
			}()
			if panicked || test.HasNegativeWeight {
				if !test.HasNegativeWeight {
					t.Errorf("%q: unexpected panic", test.Name)
				}
				if !panicked {
					t.Errorf("%q: expected panic for negative edge weight", test.Name)
				}
				continue
			}
			if pt.From().ID() != test.Query.From().ID() {
				t.Fatalf("Unexpected from node ID: got:%d want:%d", pt.From().ID(), test.Query.From().ID())
			}
			p, weight := pt.To(test.Query.To())
			if weight != test.Weight {
				t.Errorf("%q: unexpected weight from Between: got:%f want:%f",
					test.Name, weight, test.Weight)
			}
			if weight := pt.WeightTo(test.Query.To()); weight != test.Weight {
				t.Errorf("%q: unexpected weight from Weight: got:%f want:%f",
					test.Name, weight, test.Weight)
			}

			var got []int
			for _, n := range p {
				got = append(got, n.ID())
			}
			ok := len(got) == 0 && len(test.WantPaths) == 0
			for _, sp := range test.WantPaths {
				if reflect.DeepEqual(got, sp) {
					ok = true
					break
				}
			}
			if !ok {
				t.Errorf("%q: unexpected shortest path:\ngot: %v\nwant from:%v",
					test.Name, p, test.WantPaths)
			}

			np, weight := pt.To(test.NoPathFor.To())
			if pt.From().ID() == test.NoPathFor.From().ID() && (np != nil || !math.IsInf(weight, 1)) {
				t.Errorf("%q: unexpected path:\ngot: path=%v weight=%f\nwant:path=<nil> weight=+Inf",
					test.Name, np, weight)
			}
		}

	}
}
