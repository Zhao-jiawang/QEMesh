#include "Shader.h"

#include <vector>

namespace {

GLuint compile(GLenum type, const char* src, std::string* err) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(len);
        glGetShaderInfoLog(shader, len, nullptr, log.data());
        if (err) {
            *err = std::string(log.begin(), log.end());
        }
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

}

bool Shader::load(const char* vs, const char* fs, std::string* err) {
    GLuint v = compile(GL_VERTEX_SHADER, vs, err);
    if (!v) {
        return false;
    }
    GLuint f = compile(GL_FRAGMENT_SHADER, fs, err);
    if (!f) {
        glDeleteShader(v);
        return false;
    }
    program_ = glCreateProgram();
    glAttachShader(program_, v);
    glAttachShader(program_, f);
    glLinkProgram(program_);
    glDeleteShader(v);
    glDeleteShader(f);

    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(len);
        glGetProgramInfoLog(program_, len, nullptr, log.data());
        if (err) {
            *err = std::string(log.begin(), log.end());
        }
        glDeleteProgram(program_);
        program_ = 0;
        return false;
    }
    return true;
}

void Shader::destroy() {
    if (program_ != 0) {
        glDeleteProgram(program_);
        program_ = 0;
    }
}
