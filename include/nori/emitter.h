/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class Mesh;
/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
    
    virtual Color3f getRadiance() const = 0;

    virtual void setMesh(Mesh* mesh) = 0;

    virtual void generatePhoton(Sampler* sampler, Vector3f& origin, Vector3f& dir, Color3f& power, int N) = 0;
};

NORI_NAMESPACE_END
