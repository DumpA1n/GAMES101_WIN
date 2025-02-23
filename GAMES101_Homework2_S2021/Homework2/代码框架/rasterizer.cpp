// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Vector3f p(x, y, 0);
    float n1 = (_v[1] - _v[0]).cross(p - _v[0]).z();
    float n2 = (_v[2] - _v[1]).cross(p - _v[1]).z();
    float n3 = (_v[0] - _v[2]).cross(p - _v[2]).z();
    return (n1 > 0 && n2 > 0 && n3 > 0) || (n1 < 0 && n2 < 0 && n3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    static auto get_depth = [](float x, float y, const Triangle& t) -> float {
        auto v = t.toVector4();
        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        z_interpolated *= w_reciprocal;
        return z_interpolated;
    };

    static float aa_steps[4][2] = {
        {0.25, 0.25},
        {0.25, 0.75},
        {0.75, 0.25},
        {0.75, 0.75}
    };

    static const enum Antialiasing : int {
        NORMAL = 0,
        SSAA = 1,
        MSAA = 2
    };

    int AA = SSAA;

    Vector2f st(MIN(MIN(v[0].x(), v[1].x()), v[2].x()), MIN(MIN(v[0].y(), v[1].y()), v[2].y()));
    Vector2f ed(MAX(MAX(v[0].x(), v[1].x()), v[2].x()), MAX(MAX(v[0].y(), v[1].y()), v[2].y()));
    if (AA == SSAA) {
        for (int x = (int)st.x(); x < (int)ed.x(); x++) {
            for (int y = (int)st.y(); y < (int)ed.y(); y++) {
                for (int i = 0; i < 4; i++) {
                    Vector2f sp(x + aa_steps[i][0], y + aa_steps[i][1]);
                    if (insideTriangle(sp.x(), sp.y(), t.v)) {
                        float cur_depth = get_depth(sp.x(), sp.y(), t);
                        if (cur_depth < depth_buf_aa[get_index(x, y)][i]) {
                            frame_buf_aa[get_index(x, y)][i] = t.getColor();
                            depth_buf_aa[get_index(x, y)][i] = cur_depth;
                        }
                    }
                }
                set_pixel(Vector3f{(float)x, (float)y, 0.0f}, {0, 0, 0});
            }
        }
    } else if (AA == MSAA) {
        for (int x = (int)st.x(); x < (int)ed.x(); x++) {
            for (int y = (int)st.y(); y < (int)ed.y(); y++) {
                int inside_count = 0;
                for (int i = 0; i < 4; i++) {
                    if (insideTriangle(x + aa_steps[i][0], y + aa_steps[i][1], t.v)) {
                        inside_count++;
                    }
                }
                if (inside_count > 0) {
                    Vector2f sp(x + 0.5f, y + 0.5f);
                    float cur_depth = get_depth(sp.x(), sp.y(), t);
                    if (cur_depth < depth_buf[get_index(x, y)]) {
                        set_pixel(Vector3f{(float)x, (float)y, cur_depth}, t.getColor() * inside_count / 4.0f);
                        depth_buf[get_index(x, y)] = cur_depth;
                    }
                }
                if (inside_count < 4) {
                    depth_buf[get_index(x, y)] = std::numeric_limits<float>::infinity();
                }
            }
        }
    } else {
        for (int x = (int)st.x(); x < (int)ed.x(); x++) {
            for (int y = (int)st.y(); y < (int)ed.y(); y++) {
                if (insideTriangle(x, y, t.v)) {
                    float cur_depth = get_depth((float)x, (float)y, t);
                    if (cur_depth < depth_buf[x * width + y]) {
                        set_pixel(Vector3f{(float)x, (float)y, cur_depth}, t.getColor());
                        depth_buf[x * width + y] = cur_depth;
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_aa.begin(), frame_buf_aa.end(), std::vector<Vector3f>(4, Eigen::Vector3f{0, 0, 0}));
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_aa.begin(), depth_buf_aa.end(), std::vector<float>(4, std::numeric_limits<float>::infinity()));
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_aa.resize(w * h);
    for (auto& it : frame_buf_aa) {
        it.resize(4, Eigen::Vector3f{0, 0, 0});
    }
    depth_buf_aa.resize(w * h);
    for (auto& it : depth_buf_aa) {
        it.resize(4, std::numeric_limits<float>::infinity());
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

    // antialiasing
    frame_buf[ind] += (frame_buf_aa[ind][0] + frame_buf_aa[ind][1] + frame_buf_aa[ind][2] + frame_buf_aa[ind][3]) / frame_buf_aa[ind].size();
}

// clang-format on