/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
    /* Nothing to do here for now */
    m_octree = new Octree(m_bbox, m_mesh);
    /*
    std::vector<uint32_t> triangle_list;
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx)
    {
        triangle_list.emplace_back(idx);
    }
    m_octree->root = m_octree->build(m_bbox, m_mesh, triangle_list);
    */
    m_octree->Build();
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    //bool foundIntersection = false;  // Was an intersection found so far?
    //uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    //Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    ///* Brute force search through all triangles */
    //
    //for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
    //    float u, v, t;
    //    if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
    //        /* An intersection was found! Can terminate
    //           immediately if this is a shadow ray query */
    //        if (shadowRay)
    //            return true;
    //        ray.maxt = its.t = t;
    //        its.uv = Point2f(u, v);
    //        its.mesh = m_mesh;
    //        f = idx;
    //        foundIntersection = true;
    //    }
    //}

    //if (foundIntersection) {
    //    /* At this point, we now know that there is an intersection,
    //       and we know the triangle index of the closest such intersection.

    //       The following computes a number of additional properties which
    //       characterize the intersection (normals, texture coordinates, etc..)
    //    */

    //    /* Find the barycentric coordinates */
    //    Vector3f bary;
    //    bary << 1-its.uv.sum(), its.uv;

    //    /* References to all relevant mesh buffers */
    //    const Mesh *mesh   = its.mesh;
    //    const MatrixXf &V  = mesh->getVertexPositions();
    //    const MatrixXf &N  = mesh->getVertexNormals();
    //    const MatrixXf &UV = mesh->getVertexTexCoords();
    //    const MatrixXu &F  = mesh->getIndices();

    //    /* Vertex indices of the triangle */
    //    uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

    //    Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

    //    /* Compute the intersection positon accurately
    //       using barycentric coordinates */
    //    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    //    /* Compute proper texture coordinates if provided by the mesh */
    //    if (UV.size() > 0)
    //        its.uv = bary.x() * UV.col(idx0) +
    //            bary.y() * UV.col(idx1) +
    //            bary.z() * UV.col(idx2);

    //    /* Compute the geometry frame */
    //    its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

    //    if (N.size() > 0) {
    //        /* Compute the shading frame. Note that for simplicity,
    //           the current implementation doesn't attempt to provide
    //           tangents that are continuous across the surface. That
    //           means that this code will need to be modified to be able
    //           use anisotropic BRDFs, which need tangent continuity */

    //        its.shFrame = Frame(
    //            (bary.x() * N.col(idx0) +
    //             bary.y() * N.col(idx1) +
    //             bary.z() * N.col(idx2)).normalized());
    //    } else {
    //        its.shFrame = its.geoFrame;
    //    }
    //}

    //return foundIntersection;
    //uint32_t resIdx = (uint32_t)-1;
    //Ray3f ray(ray_);
    //if (rayIntersectOctree(m_octree->root, ray, its, shadowRay, resIdx))
    //{
    //    if (shadowRay)
    //        return true;

    //    Vector3f bary;
    //    bary << 1 - its.uv.sum(), its.uv;

    //    /* References to all relevant mesh buffers */
    //    const Mesh* mesh = its.mesh;
    //    const MatrixXf& V = mesh->getVertexPositions();
    //    const MatrixXf& N = mesh->getVertexNormals();
    //    const MatrixXf& UV = mesh->getVertexTexCoords();
    //    const MatrixXu& F = mesh->getIndices();

    //    /* Vertex indices of the triangle */
    //    uint32_t idx0 = F(0, resIdx), idx1 = F(1, resIdx), idx2 = F(2, resIdx);

    //    Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

    //    /* Compute the intersection positon accurately
    //        using barycentric coordinates */
    //    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    //    /* Compute proper texture coordinates if provided by the mesh */
    //    if (UV.size() > 0)
    //        its.uv = bary.x() * UV.col(idx0) +
    //        bary.y() * UV.col(idx1) +
    //        bary.z() * UV.col(idx2);

    //    /* Compute the geometry frame */
    //    its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

    //    if (N.size() > 0) {
    //        /* Compute the shading frame. Note that for simplicity,
    //            the current implementation doesn't attempt to provide
    //            tangents that are continuous across the surface. That
    //            means that this code will need to be modified to be able
    //            use anisotropic BRDFs, which need tangent continuity */

    //        its.shFrame = Frame(
    //            (bary.x() * N.col(idx0) +
    //                bary.y() * N.col(idx1) +
    //                bary.z() * N.col(idx2)).normalized());
    //    }
    //    else {
    //        its.shFrame = its.geoFrame;
    //    }
    //    return true;
    //}
    //return false;

    int AccelType = 1;
    switch (AccelType)
    {
    case 1 :
        return RayIntersectOctree(ray_, its, shadowRay);
    default:
        return RayIntersect(ray_, its, shadowRay);
    }
}

bool Accel::RayIntersect(const Ray3f& ray_, Intersection& its, bool shadowRay) const
{
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Brute force search through all triangles */
    
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

/*
bool Accel::rayIntersectOctree(const Node* InNode, Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& resIdx) const
{
    //if (InNode->bNoTriangles)
      //  return false;

    bool bFoundIntersection = false;

    if (!InNode->current_bbox.rayIntersect(ray))
        return bFoundIntersection;

    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                if (InNode->children[i][j][k])
                {
                    bFoundIntersection |= rayIntersectOctree(InNode->children[i][j][k], ray, its, shadowRay, resIdx);
                }
            }

    if (InNode->tris_index.size() > 0)
    {
        //uint32_t f = (uint32_t)-1;
        //Ray3f ray_(ray);

        for (uint32_t i = 0; i < InNode->tris_index.size(); ++i)
        {
            float u, v, t;
            uint32_t idx = InNode->tris_index[i];
            if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
                if (shadowRay)
                    return true;
                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_mesh;
                resIdx = idx;
                bFoundIntersection = true;
            }
        }
    }

    return bFoundIntersection;
}
/*
Node* Octree::build(BoundingBox3f InBBox, Mesh* InMesh, std::vector<uint32_t>& InTriangle_list)
{
    if (InTriangle_list.empty())
        return nullptr;
    
    if (InTriangle_list.size() <= 10)
    {
        Node* leaf = new Node(InBBox);
        for (int idx = 0; idx < InTriangle_list.size(); ++idx)
        {
            leaf->tris_index.emplace_back(InTriangle_list[idx]);
        }

        //leaf->bNoTriangles = false;
        
        return leaf;
    }

    std::vector<uint32_t> sub_triangle_lists[2][2][2];
    BoundingBox3f sub_bbox[2][2][2];

    Point3f center = InBBox.getCenter();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                Point3f bias = InBBox.getExtents() * 0.25f;
                float s_i = ((float)i - 0.5) * 2, s_j = ((float)j - 0.5) * 2, s_k = ((float)k - 0.5) * 2;
                bias = bias.cwiseProduct(Point3f(s_i, s_j, s_k));
                Point3f dummy = center + bias;
                Point3f min_p = dummy - bias.cwiseAbs();
                Point3f max_p = dummy + bias.cwiseAbs();
                sub_bbox[i][j][k] = BoundingBox3f(min_p, max_p);

                for (uint32_t tri = 0; tri < InTriangle_list.size(); ++tri)
                {
                    uint32_t idx = InTriangle_list[tri];

                    BoundingBox3f tri_bbox = InMesh->getBoundingBox(idx);

                    if (sub_bbox[i][j][k].contains(tri_bbox) || sub_bbox[i][j][k].overlaps(tri_bbox) || tri_bbox.contains(sub_bbox[i][j][k]))
                    {
                        sub_triangle_lists[i][j][k].emplace_back(idx);
                    }
                }
            }



    Node* child = new Node(InBBox);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                //child->bNoTriangles = false;
                child->children[i][j][k] = build(sub_bbox[i][j][k], InMesh, sub_triangle_lists[i][j][k]);
            }
    return child;
}*/

void Octree::Build()
{
    uint32_t TriangleNums = Mesh->getTriangleCount();

    std::vector<uint32_t> TriangleList;
    for (uint32_t Idx = 0; Idx < TriangleNums; ++Idx)
    {
        TriangleList.emplace_back(Idx);
    }

    if (TriangleList.size() > MAX_LEAF_TRI_NUMS)
    {
        Children[0][0][0] = Divide(CurBoundingbox, TriangleList);
    }
    else
    {
        for (uint32_t Idx = 0; Idx < TriangleList.size(); ++Idx)
        {
            TriangleIndexArray.emplace_back(TriangleList[Idx]);
        }
    }
}

Octree* Octree::Divide(BoundingBox3f InBbox, std::vector<uint32_t>& TriangleList)
{
    if (TriangleList.empty())
    {
        return nullptr;
    }

    if (TriangleList.size() <= MAX_LEAF_TRI_NUMS)
    {
        Octree* Leaf = new Octree(InBbox, Mesh);
        for (uint32_t Idx = 0; Idx < TriangleList.size(); ++Idx)
        {
            Leaf->TriangleIndexArray.emplace_back(TriangleList[Idx]);
        }
        return Leaf;
    }

    std::vector<uint32_t> SubTriangleLists[2][2][2];
    BoundingBox3f SubBoundingbox[2][2][2];

    Point3f center = InBbox.getCenter();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                Point3f bias = InBbox.getExtents() * 0.25f;
                float s_i = ((float)i - 0.5) * 2, s_j = ((float)j - 0.5) * 2, s_k = ((float)k - 0.5) * 2;
                bias = bias.cwiseProduct(Point3f(s_i, s_j, s_k));
                Point3f dummy = center + bias;
                Point3f min_p = dummy - bias.cwiseAbs();
                Point3f max_p = dummy + bias.cwiseAbs();
                SubBoundingbox[i][j][k] = BoundingBox3f(min_p, max_p);

                for (uint32_t tri = 0; tri < TriangleList.size(); ++tri)
                {
                    uint32_t idx = TriangleList[tri];

                    BoundingBox3f tri_bbox = Mesh->getBoundingBox(idx);

                    if (SubBoundingbox[i][j][k].contains(tri_bbox) || SubBoundingbox[i][j][k].overlaps(tri_bbox) || tri_bbox.contains(SubBoundingbox[i][j][k]))
                    {
                        SubTriangleLists[i][j][k].emplace_back(idx);
                    }
                }
            }

    Octree* Child = new Octree(InBbox, Mesh);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                Child->Children[i][j][k] = Divide(SubBoundingbox[i][j][k], SubTriangleLists[i][j][k]);
            }
    return Child;
}

bool Accel::RayIntersectOctreeInternal(const Octree* CurrentOctree, Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& Result) const
{
    bool bFoundIntersection = false;

    if (!CurrentOctree->CurBoundingbox.rayIntersect(ray))
        return bFoundIntersection;

    //to do: sort
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                if (CurrentOctree->Children[i][j][k])
                {
                    if (CurrentOctree->Children[i][j][k]->CurBoundingbox.squaredDistanceTo(ray.o) > ray.maxt * ray.maxt)
                        continue;

                    bFoundIntersection |= RayIntersectOctreeInternal(CurrentOctree->Children[i][j][k], ray, its, shadowRay, Result);
                }
            }

    if (CurrentOctree->TriangleIndexArray.size() > 0)
    {
        for (uint32_t i = 0; i < CurrentOctree->TriangleIndexArray.size(); ++i)
        {
            float u, v, t;
            uint32_t Idx = CurrentOctree->TriangleIndexArray[i];
            if (m_mesh->rayIntersect(Idx, ray, u, v, t)) {
                if (shadowRay)
                    return true;
                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_mesh;
                Result = Idx;
                bFoundIntersection = true;
            }
        }
    }

    return bFoundIntersection;
}

bool Accel::RayIntersectOctree(const Ray3f& ray_, Intersection& its, bool shadowRay) const
{
    uint32_t Result = (uint32_t)-1;
    Ray3f ray(ray_);
    if (RayIntersectOctreeInternal(m_octree, ray, its, shadowRay, Result))
    {
        if (shadowRay)
            return true;

        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh* mesh = its.mesh;
        const MatrixXf& V = mesh->getVertexPositions();
        const MatrixXf& N = mesh->getVertexNormals();
        const MatrixXf& UV = mesh->getVertexTexCoords();
        const MatrixXu& F = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, Result), idx1 = F(1, Result), idx2 = F(2, Result);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
            using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
            bary.y() * UV.col(idx1) +
            bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
                the current implementation doesn't attempt to provide
                tangents that are continuous across the surface. That
                means that this code will need to be modified to be able
                use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                    bary.y() * N.col(idx1) +
                    bary.z() * N.col(idx2)).normalized());
        }
        else {
            its.shFrame = its.geoFrame;
        }
        return true;
    }
    return false;
}

NORI_NAMESPACE_END

