#pragma once

#include <string>
#include <glad/glad.h>

class Shader {
public:
    Shader() = default;
    bool load(const char* vs, const char* fs, std::string* err);
    void use() const { glUseProgram(program_); }
    GLuint id() const { return program_; }
    void destroy();

private:
    GLuint program_ = 0;
};
