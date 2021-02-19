# HW2 - Shortest paths in Golang

These instructions and code files are just meant to get you started. You can
use these or roll your own. However the zip file must contain a `hw2.go` file
which contains a golang implementation of the following two functions:

* `func BellmanFord(s graph.Node, g graph.Graph) Shortest`
* `func DeltaStep(s graph.Node, g graph.Graph) Shortest`

This assignemnt makes use of the library: https://github.com/gonum/graph
See the license here: https://github.com/gonum/license

In particular it uses two structs from that library, Graph and Shortest,
representing a raw graph and a particular shortest path in the same.

If you need to understand better the format of path.Graph and path.Shortest
See: https://github.com/gonum/graph/blob/master/graph.go
and: https://github.com/gonum/graph/blob/master/path/shortest.go

## Pre-requisites

* Golang (see: https://golang.org/dl/)
* Gonum

## Initializing

You should be able to download the dependencies with:
`go mod download`

A small set of tests (currently failing) are included and can be executed
with: `go test`

## Testing with bigger/different graphs

`gonum` has packages for graph generation and includes many test graphs.

## Hints

This starter code includes an implementation of the Dijkstra shortest paths
algorithm, that you can use to test your own.
