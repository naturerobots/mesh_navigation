#include <gtest/gtest.h>
#include <mesh_layers/inflation_layer.h>
#include <lvr2/util/Synthetic.hpp>

using mesh_map::Vector;

lvr2::PMPMesh<mesh_map::Vector> genTriangle()
{
    lvr2::PMPMesh<mesh_map::Vector> mesh;
    mesh.addVertex(Vector(0, 0, 0));  // Vertex 0
    mesh.addVertex(Vector(1, 0, 0));  // Vertex 1
    mesh.addVertex(Vector(0, 1, 0));  // Vertex 2

    // Create face (triangle) using the three vertices
    mesh.addFace(
        lvr2::VertexHandle(0),
        lvr2::VertexHandle(1),
        lvr2::VertexHandle(2)
    );

    return mesh;
}


lvr2::DenseEdgeMap<float> calcEdgeWeights(const lvr2::PMPMesh<Vector>& mesh)
{
    lvr2::DenseEdgeMap<float> out;
    for (lvr2::EdgeHandle eH: mesh.edges())
    {
        auto vertices = mesh.getVerticesOfEdge(eH);
        float dist = mesh.getVertexPosition(vertices[0]).distanceFrom(mesh.getVertexPosition(vertices[1]));
        out.insert(eH, dist);
    }
    return out;
}

TEST(InflationLayer, test_wave_front_update)
{
    mesh_layers::InflationLayer instance;

    //=== Set up test data ===//
    lvr2::PMPMesh<mesh_map::Vector> mesh = genTriangle();
    lvr2::DenseVertexMap<float> distances;
    lvr2::DenseVertexMap<mesh_map::Vector> vectors;
    lvr2::DenseEdgeMap<float> edge_weights = calcEdgeWeights(mesh);
    const float max_dist = 5.0;

    // The distances map passed to waveFrontUpdate might be sparse (no values for some vertices)
    auto eh = mesh.getEdgeBetween(lvr2::VertexHandle(0), lvr2::VertexHandle(1));
    distances.insert(lvr2::VertexHandle(0), 0.0);
    distances.insert(lvr2::VertexHandle(1), edge_weights[eh.unwrap()]);
    // Vertex 3 has no distance yet

    //=== Run the test ===//
    // The distance and vector should be updated for vertex 2
    EXPECT_TRUE(instance.waveFrontUpdate(
        mesh,
        distances,
        vectors,
        max_dist,
        edge_weights,
        lvr2::VertexHandle(0),
        lvr2::VertexHandle(1),
        lvr2::VertexHandle(2)
    ));

    // The distance of vertex 2 to the origin of the wave front (vertex 0)
    // should be equal to their distance -> 1.0
    const float distance = distances[lvr2::VertexHandle(2)];
    EXPECT_FLOAT_EQ(distance, 1.0f);
}
