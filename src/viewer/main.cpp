#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <misc/cpp/imgui_stdlib.h>

#include <cstdio>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

#include "../core/Mesh.h"
#include "../core/Simplify.h"
#include "../core/Tsv.h"
#include "Math.h"
#include "Shader.h"

struct GLMesh {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    int index_count = 0;
};

static bool wireframe_mode = false;
static bool show_before = true;
static float yaw_deg = 0.0f;
static float pitch_deg = 0.0f;
static float distance_to_center = 3.0f;
static Vec3 center(0.0, 0.0, 0.0);
static bool dragging = false;
static double last_x = 0.0;
static double last_y = 0.0;

void glfw_error_cb(int, const char* desc) {
    std::fprintf(stderr, "GLFW error: %s\n", desc);
}

Vec3 compute_center(const Mesh& mesh) {
    Vec3 minp(1e30, 1e30, 1e30);
    Vec3 maxp(-1e30, -1e30, -1e30);
    for (const auto& v : mesh.vertices) {
        if (!v.valid) continue;
        minp.x = std::min(minp.x, v.p.x);
        minp.y = std::min(minp.y, v.p.y);
        minp.z = std::min(minp.z, v.p.z);
        maxp.x = std::max(maxp.x, v.p.x);
        maxp.y = std::max(maxp.y, v.p.y);
        maxp.z = std::max(maxp.z, v.p.z);
    }
    return (minp + maxp) * 0.5;
}

float compute_radius(const Mesh& mesh, const Vec3& c) {
    double r = 0.0;
    for (const auto& v : mesh.vertices) {
        if (!v.valid) continue;
        double d = length(v.p - c);
        r = std::max(r, d);
    }
    return static_cast<float>(r);
}

std::string model_name_from_path(const std::string& path) {
    std::string base = path;
    size_t slash = base.find_last_of("/\\");
    if (slash != std::string::npos) {
        base = base.substr(slash + 1);
    }
    size_t dot = base.find_last_of('.');
    if (dot != std::string::npos) {
        base = base.substr(0, dot);
    }
    return base;
}

void reset_camera_for_mesh(const Mesh& mesh) {
    center = compute_center(mesh);
    float radius = compute_radius(mesh, center);
    distance_to_center = radius > 0.0f ? radius * 2.5f : 3.0f;
    yaw_deg = 0.0f;
    pitch_deg = 0.0f;
}

void build_render_data(const Mesh& mesh, std::vector<float>& vertices, std::vector<unsigned int>& indices) {
    std::vector<int> vmap(mesh.vertices.size(), -1);
    int idx = 0;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (mesh.vertices[i].valid) {
            vmap[i] = idx++;
        }
    }
    std::vector<Vec3> normals(mesh.vertices.size(), Vec3(0.0, 0.0, 0.0));
    for (const auto& f : mesh.faces) {
        if (!f.valid) continue;
        Vec3 n = face_normal(mesh, f);
        for (int k = 0; k < 3; ++k) {
            normals[f.v[k]] += n;
        }
    }

    vertices.clear();
    vertices.reserve(idx * 6);
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (!mesh.vertices[i].valid) continue;
        Vec3 p = mesh.vertices[i].p;
        Vec3 n = normalize(normals[i]);
        vertices.push_back(static_cast<float>(p.x));
        vertices.push_back(static_cast<float>(p.y));
        vertices.push_back(static_cast<float>(p.z));
        vertices.push_back(static_cast<float>(n.x));
        vertices.push_back(static_cast<float>(n.y));
        vertices.push_back(static_cast<float>(n.z));
    }

    indices.clear();
    indices.reserve(mesh.faces.size() * 3);
    for (const auto& f : mesh.faces) {
        if (!f.valid) continue;
        indices.push_back(static_cast<unsigned int>(vmap[f.v[0]]));
        indices.push_back(static_cast<unsigned int>(vmap[f.v[1]]));
        indices.push_back(static_cast<unsigned int>(vmap[f.v[2]]));
    }
}

void upload_mesh(GLMesh& glmesh, const std::vector<float>& vertices, const std::vector<unsigned int>& indices) {
    if (glmesh.vao == 0) {
        glGenVertexArrays(1, &glmesh.vao);
        glGenBuffers(1, &glmesh.vbo);
        glGenBuffers(1, &glmesh.ebo);
    }
    glBindVertexArray(glmesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, glmesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glmesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
    glmesh.index_count = static_cast<int>(indices.size());
}

void destroy_mesh(GLMesh& glmesh) {
    if (glmesh.ebo) glDeleteBuffers(1, &glmesh.ebo);
    if (glmesh.vbo) glDeleteBuffers(1, &glmesh.vbo);
    if (glmesh.vao) glDeleteVertexArrays(1, &glmesh.vao);
    glmesh = {};
}

void mouse_button_cb(GLFWwindow* window, int button, int action, int) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse) {
            dragging = true;
            glfwGetCursorPos(window, &last_x, &last_y);
        } else if (action == GLFW_RELEASE) {
            dragging = false;
        }
    }
}

void cursor_pos_cb(GLFWwindow* window, double x, double y) {
    if (!dragging) return;
    if (ImGui::GetIO().WantCaptureMouse) return;
    double dx = x - last_x;
    double dy = y - last_y;
    last_x = x;
    last_y = y;
    yaw_deg += static_cast<float>(dx * 0.5);
    pitch_deg += static_cast<float>(dy * 0.5);
    if (pitch_deg > 89.0f) pitch_deg = 89.0f;
    if (pitch_deg < -89.0f) pitch_deg = -89.0f;
}

void scroll_cb(GLFWwindow*, double, double yoff) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    distance_to_center *= (yoff > 0.0) ? 0.9f : 1.1f;
    if (distance_to_center < 0.01f) distance_to_center = 0.01f;
}

Vec3 camera_pos() {
    double yaw = yaw_deg * 3.1415926535 / 180.0;
    double pitch = pitch_deg * 3.1415926535 / 180.0;
    Vec3 dir(
        std::cos(pitch) * std::sin(yaw),
        std::sin(pitch),
        std::cos(pitch) * std::cos(yaw)
    );
    return center + dir * distance_to_center;
}

bool load_mesh(const std::string& path, Mesh& mesh, std::string* err) {
    std::string load_err;
    if (!mesh.load_obj(path, &load_err)) {
        if (err) {
            *err = load_err;
        }
        return false;
    }
    reset_camera_for_mesh(mesh);
    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: mesh_viewer input.obj [output.obj]\n");
        return 1;
    }
    std::string input_path = argv[1];
    std::string output_path = (argc >= 3) ? argv[2] : "out.obj";

    glfwSetErrorCallback(glfw_error_cb);
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to init GLFW.\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "mesh_viewer", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create window.\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::fprintf(stderr, "Failed to load GL.\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glfwSetMouseButtonCallback(window, mouse_button_cb);
    glfwSetCursorPosCallback(window, cursor_pos_cb);
    glfwSetScrollCallback(window, scroll_cb);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    const char* vs = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        uniform mat4 uMVP;
        uniform mat4 uModel;
        out vec3 vNormal;
        void main() {
            vNormal = mat3(uModel) * aNormal;
            gl_Position = uMVP * vec4(aPos, 1.0);
        }
    )";
    const char* fs = R"(
        #version 330 core
        in vec3 vNormal;
        out vec4 FragColor;
        void main() {
            vec3 n = normalize(vNormal);
            vec3 light = normalize(vec3(0.4, 0.7, 0.5));
            float diff = max(dot(n, light), 0.1);
            vec3 color = vec3(0.2, 0.8, 0.6) * diff + vec3(0.1);
            FragColor = vec4(color, 1.0);
        }
    )";

    Shader shader;
    std::string shader_err;
    if (!shader.load(vs, fs, &shader_err)) {
        std::fprintf(stderr, "Shader error: %s\n", shader_err.c_str());
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    Mesh input_mesh;
    Mesh output_mesh;
    std::string load_err;
    if (!load_mesh(input_path, input_mesh, &load_err)) {
        std::fprintf(stderr, "%s\n", load_err.c_str());
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    GLMesh gl_input;
    GLMesh gl_output;
    std::vector<float> vbo;
    std::vector<unsigned int> ibo;
    build_render_data(input_mesh, vbo, ibo);
    upload_mesh(gl_input, vbo, ibo);

    float ratio = 0.5f;
    std::string csv_path = "results.tsv";
    std::string model_name = model_name_from_path(input_path);
    SimplifyStats stats;
    bool has_output = false;

    int prev_w = 0, prev_s = 0, prev_o = 0, prev_r = 0, prev_b = 0;

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        int key_w = glfwGetKey(window, GLFW_KEY_W);
        int key_s = glfwGetKey(window, GLFW_KEY_S);
        int key_o = glfwGetKey(window, GLFW_KEY_O);
        int key_r = glfwGetKey(window, GLFW_KEY_R);
        int key_b = glfwGetKey(window, GLFW_KEY_B);
        if (key_w == GLFW_PRESS && prev_w == GLFW_RELEASE) {
            wireframe_mode = !wireframe_mode;
        }
        if (key_b == GLFW_PRESS && prev_b == GLFW_RELEASE) {
            show_before = !show_before;
        }
        if (key_r == GLFW_PRESS && prev_r == GLFW_RELEASE) {
            const Mesh& current = (show_before || !has_output) ? input_mesh : output_mesh;
            reset_camera_for_mesh(current);
        }
        if (key_o == GLFW_PRESS && prev_o == GLFW_RELEASE && has_output) {
            Mesh tmp;
            if (tmp.load_obj(output_path, nullptr)) {
                output_mesh = tmp;
                build_render_data(output_mesh, vbo, ibo);
                upload_mesh(gl_output, vbo, ibo);
                show_before = false;
            }
        }
        if (key_s == GLFW_PRESS && prev_s == GLFW_RELEASE) {
            output_mesh = input_mesh;
            std::string simp_err;
            if (simplify_mesh(output_mesh, ratio, &stats, &simp_err)) {
                output_mesh.save_obj(output_path, nullptr);
                build_render_data(output_mesh, vbo, ibo);
                upload_mesh(gl_output, vbo, ibo);
                has_output = true;
                show_before = false;
                std::string csv_err;
                if (!append_tsv(csv_path, model_name, ratio, stats, &csv_err)) {
                    std::fprintf(stderr, "%s\n", csv_err.c_str());
                }
            }
        }
        prev_w = key_w;
        prev_s = key_s;
        prev_o = key_o;
        prev_r = key_r;
        prev_b = key_b;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Mesh Simplifier");
        ImGui::InputText("Input", &input_path);
        ImGui::InputText("Output", &output_path);
        ImGui::InputText("Model", &model_name);
        ImGui::InputText("TSV", &csv_path);
        ImGui::SliderFloat("Ratio", &ratio, 0.05f, 1.0f);
        if (ImGui::Button("Simplify (S)")) {
            output_mesh = input_mesh;
            std::string simp_err;
            if (simplify_mesh(output_mesh, ratio, &stats, &simp_err)) {
                output_mesh.save_obj(output_path, nullptr);
                build_render_data(output_mesh, vbo, ibo);
                upload_mesh(gl_output, vbo, ibo);
                has_output = true;
                show_before = false;
                std::string csv_err;
                if (!append_tsv(csv_path, model_name, ratio, stats, &csv_err)) {
                    std::fprintf(stderr, "%s\n", csv_err.c_str());
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload Input")) {
            if (load_mesh(input_path, input_mesh, nullptr)) {
                build_render_data(input_mesh, vbo, ibo);
                upload_mesh(gl_input, vbo, ibo);
                show_before = true;
                model_name = model_name_from_path(input_path);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload Output (O)") && has_output) {
            Mesh tmp;
            if (tmp.load_obj(output_path, nullptr)) {
                output_mesh = tmp;
                build_render_data(output_mesh, vbo, ibo);
                upload_mesh(gl_output, vbo, ibo);
                show_before = false;
            }
        }

        ImGui::Text("Show: %s (B)", show_before ? "Before" : "After");
        ImGui::Text("Wireframe: %s (W)", wireframe_mode ? "On" : "Off");
        ImGui::Text("Reset Camera: R");
        ImGui::Separator();
        ImGui::Text("Input V/F: %d / %d", input_mesh.valid_vertex_count(), input_mesh.valid_face_count());
        if (has_output) {
            ImGui::Text("Output V/F: %d / %d", output_mesh.valid_vertex_count(), output_mesh.valid_face_count());
            ImGui::Text("Collapsed: %d", stats.collapsed_edges);
            ImGui::Text("Skipped invalid: %d", stats.skipped_invalid);
            ImGui::Text("Non-manifold rejects: %d", stats.nonmanifold_rejects);
            ImGui::Text("Time total: %.2f ms", stats.time_total_ms);
            ImGui::Text("Build: %.2f ms", stats.time_build_ms);
            ImGui::Text("Simplify: %.2f ms", stats.time_simplify_ms);
            ImGui::Text("QEM mean/max: %.6f / %.6f", stats.qem_err_mean, stats.qem_err_max);
            ImGui::Text("Degenerate faces: %d", stats.degenerate_faces);
            ImGui::Text("Aspect mean: %.4f", stats.aspect_mean);
            ImGui::Text("Consistency OK: %s", stats.consistency_ok ? "Yes" : "No");
        }
        ImGui::End();

        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.08f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glPolygonMode(GL_FRONT_AND_BACK, wireframe_mode ? GL_LINE : GL_FILL);

        Mat4 proj = Mat4::perspective(45.0f * 3.1415926535f / 180.0f, (float)w / (float)h, 0.01f, 1000.0f);
        Vec3 eye = camera_pos();
        Mat4 view = Mat4::look_at(eye, center, Vec3(0.0, 1.0, 0.0));
        Mat4 model = Mat4::identity();
        Mat4 mvp = proj * view * model;

        shader.use();
        GLint loc_mvp = glGetUniformLocation(shader.id(), "uMVP");
        GLint loc_model = glGetUniformLocation(shader.id(), "uModel");
        glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, mvp.m);
        glUniformMatrix4fv(loc_model, 1, GL_FALSE, model.m);

        const GLMesh& draw_mesh = (show_before || !has_output) ? gl_input : gl_output;
        glBindVertexArray(draw_mesh.vao);
        glDrawElements(GL_TRIANGLES, draw_mesh.index_count, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    destroy_mesh(gl_input);
    destroy_mesh(gl_output);
    shader.destroy();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
