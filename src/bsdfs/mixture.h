/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"

TR_NAMESPACE_BEGIN

/**
 * Modified Phong reflectance model + Diffuse
 */
struct MixtureBSDF : BSDF {
    std::unique_ptr<Texture < v3f>> specularReflectance;
    std::unique_ptr<Texture < v3f>> diffuseReflectance;
    std::unique_ptr<Texture < float>> exponent;
    float specularSamplingWeight;
    float scale;

    MixtureBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
        const tinyobj::material_t& mat = scene.materials[matID];

        if (mat.specular_texname.empty())
            specularReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.specular)));
        else
            specularReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.specular_texname));

        if (mat.diffuse_texname.empty())
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.diffuse)));
        else
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.diffuse_texname));

        exponent = std::unique_ptr<Texture<float>>(new ConstantTexture1f(mat.shininess));

        //get scale value to ensure energy conservation
        v3f maxValue = specularReflectance->getMax() + diffuseReflectance->getMax();
        float actualMax = max(max(maxValue.x, maxValue.y), maxValue.z);
        scale = actualMax > 1.0f ? 0.99f * (1.0f / actualMax) : 1.0f;

        float dAvg = getLuminance(diffuseReflectance->getAverage() * scale);
        float sAvg = getLuminance(specularReflectance->getAverage() * scale);
        specularSamplingWeight = sAvg / (dAvg + sAvg);

        components.push_back(EGlossyReflection);
        components.push_back(EDiffuseReflection);

        combinedType = 0;
        for (unsigned int component : components)
            combinedType |= component;
    }

    inline v3f reflect(const v3f& d) const {
        return v3f(-d.x, -d.y, d.z);
    }

    v3f eval(const SurfaceInteraction& i) const override {
        v3f val(0.f);

        if(i.frameNg.cosTheta(i.wi) > 0 && i.frameNg.cosTheta(i.wo) > 0){
            v3f diffuse(diffuseReflectance->eval(worldData, i));
            v3f specular(specularReflectance->eval(worldData, i));
            float expo = exponent->eval(worldData, i);
            float ang = glm::angle(reflect(i.wi), i.wo);
            float cosAlpha = std::pow(std::max(0.f, cos(std::max(0.f, ang))), expo);

            val = scale * (diffuse / M_PI + specular * (expo + 2.f) * INV_TWOPI * cosAlpha) * i.frameNs.cosTheta(i.wi); //TODO: Check if I need to remove scale
            //val = scale * (specular * (expo + 2.f) * INV_TWOPI * cosAlpha) * i.frameNs.cosTheta(i.wi);
        }

        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {
        float pdf = 0.f;
        float phongPdf = 0.f;
        float bsdfPdf = 0.f;

        v3f reflectDir = reflect(i.wo);
        v3f dir = glm::toMat4(glm::quat(reflectDir, v3f(0.0f, 0.0f, 1.0f))) * v4f(i.wi, 1.f);
        dir = glm::normalize(dir);

        phongPdf = fmax(Warp::squareToPhongLobePdf(dir, exponent->eval(worldData, i)), 0.f);

        bsdfPdf = fmax(Warp::squareToCosineHemispherePdf(i.wi), 0.f);

        return phongPdf * specularSamplingWeight + bsdfPdf * (1 - specularSamplingWeight);
    }

    v3f sample(SurfaceInteraction& i, const v2f& _sample, float* pdf) const override {
        v3f val(0.f);

        //if diffuse
        if(_sample.x > specularSamplingWeight){  //TODO: Check if the less than is reversed
            v2f redistributedSample = v2f((_sample.x - specularSamplingWeight)/(1.f - specularSamplingWeight), _sample.y);
            glm::vec3 sampleDir = glm::normalize(Warp::squareToCosineHemisphere(redistributedSample));
            i.wi = sampleDir;
        }
        //else phong
        else{
            v2f redistributedSample = v2f(_sample.x / specularSamplingWeight, _sample.y);
            glm::vec3 sampleDir = glm::normalize(Warp::squareToPhongLobe(redistributedSample, exponent->eval(worldData, i)));
            v3f reflectDir = reflect(i.wo);
            i.wi = glm::toMat4(glm::quat(v3f(0.0f, 0.0f, 1.0f), reflectDir)) * v4f(sampleDir, 1.f);
            i.wi = glm::normalize(i.wi);
        }

        *pdf = MixtureBSDF::pdf(i);

        if(*pdf != 0)
            val = MixtureBSDF::eval(i) / *pdf;

        return val;
    }

    std::string toString() const override { return "Mixture"; }
};

TR_NAMESPACE_END