/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

struct Node;
class Octree;
/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();

    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

    //bool rayIntersectOctree(const Node* InNode, Ray3f &ray, Intersection& its, bool shadowRay, uint32_t& resIdx) const;

    bool RayIntersectOctreeInternal(const Octree* CurrentOctree, Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& Result) const;
    bool RayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay) const;
    bool RayIntersectOctree(const Ray3f& ray, Intersection& its, bool shadowRay) const;

private:
    Mesh         *m_mesh = nullptr; ///< Mesh (only a single one for now)
    BoundingBox3f m_bbox;           ///< Bounding box of the entire scene

    Octree* m_octree = nullptr;
};

struct Node
{
    struct Node* children[2][2][2];
    //uint32_t tris_index[10];
    std::vector<uint32_t> tris_index;
    BoundingBox3f current_bbox;
    bool bNoTriangles = true;

    Node(Node& rhs)
    {
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    children[i][j][k] = rhs.children[i][j][k];

        for (uint32_t i = 0; i < 10; ++i)
            tris_index[i] = rhs.tris_index[i];

        bNoTriangles = rhs.bNoTriangles;
        current_bbox = rhs.current_bbox;
    }

    Node(BoundingBox3f InBoundingBox)
        : current_bbox(InBoundingBox)
    {
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    children[i][j][k] = nullptr;
    }
};

class Octree
{
public:
    Octree (Octree& rhs)
    {
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    Children[i][j][k] = rhs.Children[i][j][k];

        for (uint32_t Idx = 0; Idx < rhs.TriangleIndexArray.size(); ++Idx)
        {
            TriangleIndexArray.emplace_back(rhs.TriangleIndexArray[Idx]);
        }

        CurBoundingbox = rhs.CurBoundingbox;
        Mesh = rhs.Mesh;
    }

    Octree(BoundingBox3f InBbox, Mesh* InMesh)
        : Mesh(InMesh)
        , CurBoundingbox(InBbox)
    {
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    Children[i][j][k] = nullptr;
    };

    class Octree* Children[2][2][2];
    Mesh* Mesh;
    BoundingBox3f CurBoundingbox;
    std::vector<uint32_t> TriangleIndexArray;

    void Build();
    Octree* Divide(BoundingBox3f InBbox, std::vector<uint32_t>& TriangleList);

    const uint32_t MAX_LEAF_TRI_NUMS = 30;
};
NORI_NAMESPACE_END
