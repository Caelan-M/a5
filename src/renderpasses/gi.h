/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include <core/core.h>
#include "core/renderpass.h"
#include "tiny_obj_loader.h"
#include "integrators/path.h"

TR_NAMESPACE_BEGIN

/**
 * Global Illumination baking renderpass.
 */
struct GIPass : RenderPass {
    GLuint shader{0};

    GLuint modelMatUniform{0};
    GLuint viewMatUniform{0};
    GLuint projectionMatUniform{0};

    int m_samplePerVertex;

    std::unique_ptr<PathTracerIntegrator> m_ptIntegrator;

    explicit GIPass(const Scene& scene) : RenderPass(scene) {
        m_ptIntegrator = std::unique_ptr<PathTracerIntegrator>(new PathTracerIntegrator(scene));
        m_ptIntegrator->m_maxDepth = scene.config.integratorSettings.gi.maxDepth;
        m_ptIntegrator->m_rrProb = scene.config.integratorSettings.gi.rrProb;
        m_ptIntegrator->m_rrDepth = scene.config.integratorSettings.gi.rrDepth;
        m_samplePerVertex = scene.config.integratorSettings.gi.samplesByVertex;
    }

    virtual void buildVBO(size_t objectIdx) override {
        GLObject& obj = objects[objectIdx];

        // TODO: Implement this
        const tinyobj::attrib_t& sa = scene.worldData.attrib;
        const tinyobj::shape_t& s = scene.worldData.shapes[objectIdx];

        obj.nVerts = s.mesh.indices.size();
        obj.vertices.resize(obj.nVerts * N_ATTR_PER_VERT);

        int k = 0;
        for (size_t i = 0; i < s.mesh.indices.size(); i++) {
            int idx = s.mesh.indices[i].vertex_index;
            int idx_n = s.mesh.indices[i].normal_index;

            // Position
            obj.vertices[k + 0] = sa.vertices[3 * idx + 0];
            obj.vertices[k + 1] = sa.vertices[3 * idx + 1];
            obj.vertices[k + 2] = sa.vertices[3 * idx + 2];
            v3f pos = v3f(sa.vertices[3 * idx + 0], sa.vertices[3 * idx + 1], sa.vertices[3 * idx + 2]);

            // Normal
            float nx = sa.normals[3 * idx_n + 0];
            float ny = sa.normals[3 * idx_n + 1];
            float nz = sa.normals[3 * idx_n + 2];
            float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
            v3f n = v3f(nx / norm, ny / norm, nz / norm);
//            obj.vertices[k + 3] = nx / norm;
//            obj.vertices[k + 4] = ny / norm;
//            obj.vertices[k + 5] = nz / norm;

            //Colour
            SurfaceInteraction surfInt;
            surfInt.primID = scene.getPrimitiveID(i);  //TODO: Check if I is correct
            surfInt.matID = scene.getMaterialID(objectIdx, surfInt.primID);
            surfInt.shapeID = objectIdx;
            surfInt.p = pos + n * Epsilon;
            surfInt.frameNs = Frame(n);
            surfInt.frameNg = Frame(n);

            Sampler sampler(260744278);

            v3f sampleDir = Warp::squareToUniformSphere(sampler.next2D());

            Ray ray(pos, sampleDir);

            //surfInt.wo = surfInt.frameNs.toLocal(-ray.d);
            surfInt.wo = v3f(0.f, 0.f, 1.f);

            v3f color = v3f(0.f);
            for(int j = 0; j < m_samplePerVertex; j++){
                color += m_ptIntegrator->renderExplicit(ray, sampler, surfInt);
            }
            color /= m_samplePerVertex;

//            for(int j = 0; j < 4; j++){
//                color += m_ptIntegrator->renderExplicit(ray, sampler, surfInt);
//            }
//            color /= 4;


            obj.vertices[k + 3] = color.r;
            obj.vertices[k + 4] = color.g;
            obj.vertices[k + 5] = color.b;

            k += N_ATTR_PER_VERT;
        }

        // VBO
        glGenVertexArrays(1, &obj.vao);
        glBindVertexArray(obj.vao);

        glGenBuffers(1, &obj.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, obj.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * obj.nVerts * N_ATTR_PER_VERT,
                     (GLvoid*) (&obj.vertices[0]),
                     GL_STATIC_DRAW);
    }

    bool init(const Config& config) override {
        RenderPass::init(config);

        // Create shader
        GLuint vs = compileShader("gi.vs", GL_VERTEX_SHADER);
        GLuint fs = compileShader("gi.fs", GL_FRAGMENT_SHADER);
        shader = compileProgram(vs, fs);
        glDeleteShader(vs);
        glDeleteShader(fs);

        // Create uniforms
        modelMatUniform = GLuint(glGetUniformLocation(shader, "model"));
        viewMatUniform = GLuint(glGetUniformLocation(shader, "view"));
        projectionMatUniform = GLuint(glGetUniformLocation(shader, "projection"));

        // Create vertex buffers
        objects.resize(scene.worldData.shapes.size());
        for (size_t i = 0; i < objects.size(); i++) {
            buildVBO(i);
            buildVAO(i);
        }

        return true;
    }

    void cleanUp() override {
        // Delete vertex buffers
        for (size_t i = 0; i < objects.size(); i++) {
            glDeleteBuffers(1, &objects[i].vbo);
            glDeleteVertexArrays(1, &objects[i].vao);
        }

        RenderPass::cleanUp();
    }

    void render() override {
        glBindFramebuffer(GL_FRAMEBUFFER, postprocess_fboScreen);
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        // Update camera
        glm::mat4 model, view, projection;
        camera.Update();
        camera.GetMatricies(projection, view, model);

        //1) Use the shader for the geometry pass.
        glUseProgram(shader);

        //2) Pass the necessary uniforms.
        GLuint modelUniform = GLuint(glGetUniformLocation(shader, "model"));
        GLuint viewUniform = GLuint(glGetUniformLocation(shader, "view"));
        GLuint projectionUniform = GLuint(glGetUniformLocation(shader, "projection"));
        glUniformMatrix4fv(modelUniform, 1, GL_FALSE, &(model[0][0]));
        glUniformMatrix4fv(viewUniform, 1, GL_FALSE, &(view[0][0]));
        glUniformMatrix4fv(projectionUniform, 1, GL_FALSE, &(projection[0][0]));

        //3) Bind vertex array of current object.
        //4) Draw its triangles.
        //5) Unbind the vertex array.
        for (size_t i = 0; i < objects.size(); i++) {
            GLObject obj = objects[i];

            glBindVertexArray(obj.vbo);
            glDrawArrays(GL_TRIANGLES, 0, obj.nVerts);
            glBindVertexArray(0);

        }

        RenderPass::render();
    }

};

TR_NAMESPACE_END
