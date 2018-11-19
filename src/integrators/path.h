/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Path tracer integrator
 */
struct PathTracerIntegrator : Integrator {
    explicit PathTracerIntegrator(const Scene& scene) : Integrator(scene) {
        m_isExplicit = scene.config.integratorSettings.pt.isExplicit;
        m_maxDepth = scene.config.integratorSettings.pt.maxDepth;
        m_rrDepth = scene.config.integratorSettings.pt.rrDepth;
        m_rrProb = scene.config.integratorSettings.pt.rrProb;
    }

    static inline float balanceHeuristic(float nf, float fPdf, float ng, float gPdf) {
        float f = nf * fPdf * fPdf, g = ng * gPdf * gPdf;
        return f / (f + g);
    }

    v3f renderImplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
        v3f Li(1.f);
        int j = 0;

        while(j <= m_maxDepth) {
            float pdf;
            glm::vec3 val(0.f);

            if(getEmission(hit) != v3f(0.f) && glm::dot(v3f(0.f, 0.f, 1.f), hit.wo) > 0) //TODO: change the v3f to the n of light
                return Li * getEmission(hit);
            else{
                val = getBSDF(hit)->sample(hit, sampler.next2D(), &pdf);

                Li *= val;

                glm::vec3 sampleDir = hit.frameNs.toWorld(hit.wi);

                //check if point light is visible from point
                Ray sampleRay(hit.p, sampleDir);

                if(!scene.bvh->intersect(sampleRay, hit))
                    return v3f(0.f);
            }
            j++;
        }
        Li = v3f(0.f);
        return Li;
    }

    v3f directLighting(Sampler& sampler, SurfaceInteraction& hit) const {
        v3f Lb(0.f);
        //Use 1 since doing multiple spp, dont need multiple samples here
        int m_emitterSamples = 1;
        int m_bsdfSamples = 1;

        for(int j = 0; j < m_bsdfSamples; j++) {
            //SurfaceInteraction surfInt;
            //SurfaceInteraction i;
            float pdf;
            glm::vec3 val(0.f);

            v2f sample = sampler.next2D();

            val = getBSDF(hit)->sample(hit, sample, &pdf);

            glm::vec3 sampleDir = hit.frameNs.toWorld(hit.wi);

            //check if point light is visible from point
            Ray sampleRay(hit.p, sampleDir);

            SurfaceInteraction i;
            if(scene.bvh->intersect(sampleRay, i)){

                //if visible to light find its color
                if(getEmission(i) != v3f(0.f)) {


                    float emPdf = 1.f/scene.emitters.size();
                    const Emitter &em = getEmitterByID(getEmitterIDByShapeID(i.shapeID));
                    v3f position = scene.getShapeCenter(em.shapeID);
                    v3f intensity = em.getRadiance();
                    float saPdf;
                    v3f wiW;
                    v3f emitterCenter = scene.getShapeCenter(em.shapeID);
                    float emitterRadius = scene.getShapeRadius(em.shapeID);
                    v3f n;
                    v3f pos;

                    sampleEmitterPosition(sampler, em, n, pos, saPdf);
                    v3f emDir = glm::normalize(pos - hit.p);
                    float cosFact = glm::dot(-emDir, n);

                    float bal = balanceHeuristic(m_bsdfSamples, pdf, m_emitterSamples, saPdf * emPdf / cosFact * glm::distance2(hit.p, pos));

                    Lb += val * getEmission(i) * bal;
                }
            }

        }
        if(m_bsdfSamples != 0)
            Lb /= m_bsdfSamples;

        v3f Lsa = v3f(0.f);

        //solid angle
        for (int j = 0; j < m_emitterSamples; j++) {

            glm::vec3 directLight(0.f);
            SurfaceInteraction i;
            float pdf = 0.f;
            float emPdf;
            size_t id = selectEmitter(sampler.next(), emPdf);
            const Emitter &em = getEmitterByID(id);
            v3f intensity = em.getRadiance();
            v3f n;
            v3f pos;
            sampleEmitterPosition(sampler, em, n, pos, pdf);

            v3f emDir = glm::normalize(pos - hit.p);
            hit.wi = hit.frameNs.toLocal(emDir);

            Ray sampleRay(hit.p, emDir);

            if (scene.bvh->intersect(sampleRay, i)) {
                if (getEmission(i) != v3f(0.f)) {
                    float cosFact = max(0.f, glm::dot(-emDir, n));
                    float cosFact1 = glm::dot(-emDir, n);
                    intensity = getEmission(i) / glm::distance2(hit.p, pos);
                    v3f val = getBSDF(hit)->eval(hit);

                    float bsdfPdf = getBSDF(hit)->pdf(hit);
                    float bal = balanceHeuristic(m_emitterSamples, pdf * emPdf / cosFact1 * glm::distance2(hit.p, pos), m_bsdfSamples, bsdfPdf);

                    Lsa = intensity * val * bal / pdf / emPdf * cosFact;
                }
            }
        }
        if(m_emitterSamples != 0)
            Lsa /= m_emitterSamples;

        return Lsa + Lb;
    }

    v3f indirectLighting(Sampler& sampler, SurfaceInteraction& hit, int depth) const {
        v3f Li(1.f);
        SurfaceInteraction i;

        if(getEmission(hit) != v3f(0.f) && depth == 0)
            return getEmission(hit);

        depth++;
        if(m_maxDepth == -1) {
            if(depth > m_rrDepth && sampler.next() > m_rrProb)
                return v3f(0.f);
        }
        else if(depth >= m_maxDepth)
            return v3f(0.f);

        float pdf;
        glm::vec3 indirectLight(0.f);

        v3f emission = v3f(200.f);
        int j = 0;
        while(emission != v3f(0.f)) {
            indirectLight = getBSDF(hit)->sample(hit, sampler.next2D(), &pdf);

            glm::vec3 sampleDir = hit.frameNs.toWorld(hit.wi);

            //check if point light is visible from point
            Ray sampleRay(hit.p, sampleDir);

            if (!scene.bvh->intersect(sampleRay, i))
                return v3f(0.f);
            if(j >= 5) //avoid getting stuck inside this loop too long, adds bias but would happen in MIS
                return v3f(0.f);
            emission = getEmission(i);
            j++;
        }

        if(m_maxDepth == -1)
            Li *= indirectLight / m_rrProb * (indirectLighting(sampler, i, depth) + directLighting(sampler, i));
        else
            Li *= indirectLight * (indirectLighting(sampler, i, depth) + directLighting(sampler, i));

        return Li;
    }

    v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
        v3f indirect(0.f);
        v3f direct(0.f);

        indirect = indirectLighting(sampler, hit, 0);

        if(m_maxDepth != 0)
            direct = directLighting(sampler, hit);

        return direct + indirect;
    }

    v3f render(const Ray& ray, Sampler& sampler) const override {
        Ray r = ray;
        SurfaceInteraction hit;

        if (scene.bvh->intersect(r, hit)) {
            if (m_isExplicit)
                return this->renderExplicit(ray, sampler, hit);
            else
                return this->renderImplicit(ray, sampler, hit);
        }
        return v3f(0.0);
    }

    int m_maxDepth;     // Maximum number of bounces
    int m_rrDepth;      // When to start Russian roulette
    float m_rrProb;     // Russian roulette probability
    bool m_isExplicit;  // Implicit or explicit
};

TR_NAMESPACE_END
