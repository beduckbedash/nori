#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
public:
	SimpleIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
		m_pointLightPosition = props.getPoint("position");
		m_pointLightEnergy = props.getColor("energy");
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		if (m_radius < .1f)
		{
			Color3f Radiance(0.f);
			Point3f Position(its.p);
			Vector3f Wi = (m_pointLightPosition - Position).normalized();
			float Vis = 1.f;
			if (scene->rayIntersect(Ray3f(Position, Wi), its))
				Vis = 0.f;

			float CosineTheta = its.shFrame.n.dot(Wi);
			Radiance += Vis * (m_pointLightEnergy / (4.f * M_PI * M_PI)) * std::max(0.f, CosineTheta) / (Position - m_pointLightPosition).dot(Position - m_pointLightPosition);
			return Radiance;
		}
		else
		{
			Color3f Radiance(0.f);
			Point3f Position(its.p);
			Point2f PositionSample = sampler->next2D();
			//Vector3f SpherePosition = Warp::squareToUniformSphere(PositionSample);
			//float Pdf = Warp::squareToUniformSpherePdf(SpherePosition);
			//SpherePosition += m_pointLightPosition;
			//Vector3f VectorToSpherePosition = SpherePosition - Position;
			//Vector3f Wi = (VectorToSpherePosition).normalized();
			//float SquareDistance = (VectorToSpherePosition).dot(VectorToSpherePosition);

			Vector3f SpherePosition = Warp::squareToUniformHemisphere(PositionSample);
			float Pdf = Warp::squareToUniformHemispherePdf(SpherePosition);
			Vector3f Wi = (SpherePosition).normalized();

			float Vis = 0.f;
			if (scene->rayIntersect(Ray3f(Position, Wi), its))//ray intersect sphere
			{
				Vector3f Vec = its.p - SpherePosition;
				Vector3f PosToSphere = m_pointLightPosition - Position;
				float LengthSquare = PosToSphere.dot(PosToSphere);
				if (LengthSquare <= Vec.dot(PosToSphere.normalized()))
					Vis = 1.f;
			}
			float CosineTheta = its.shFrame.n.dot(Wi);
			Radiance += Vis * (m_pointLightEnergy / (4.f * M_PI * M_PI)) * std::max(0.f, CosineTheta) / (Position - SpherePosition).dot(Position - SpherePosition) / Pdf;
			return Radiance;
		}
	}

	/*
	std::string toString() const {
		return tfm::format(
			"NormalIntegrator[\n"
			" myProperty = \"%s\"\n"
			"]",
			m_myProperty
		);
	}
	*/

	std::string toString() const {
		return "SimpleIntegrator[]";
	}

protected:
	std::string m_myProperty;
	Color3f m_pointLightEnergy;
	Point3f m_pointLightPosition;
	float m_radius = 10.f;
};


NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END